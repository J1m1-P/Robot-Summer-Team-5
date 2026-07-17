#include <Arduino.h>
#include <math.h>

#include <WiFi.h>
#include <WebSocketsServer.h>

#include "config/drivetrain_velocity_kinematics_config.h"
#include "config/encoder_config.h"
#include "config/motor_config.h"
#include "config/wheel_velocity_pi_config.h"
#include "control/drivetrain_velocity_kinematics.h"
#include "control/wheel_velocity_pi.h"
#include "drivers/encoder_driver.h"
#include "drivers/motor_driver.h"

// Full-robot (all 4 wheels) drive test -- see platformio.ini's
// [env:drive]. Battery powers the motors; USB is for serial
// control/telemetry only, not motor power -- board runs off USB, drive
// battery runs the motors as normal. Separate from tuning_main.cpp (single
// wheel, open-loop-friendly commands) and main.cpp (production).
//
// This is the first thing in the repo that actually drives the chassis
// using control/drivetrain_velocity_kinematics.h -- that model's per-wheel
// corner signs have only been checked against the old duty mixer
// (test_kinematics_parity) and hand math, never against a real robot
// actually moving. Test cautiously: try "drive 0" (forward) first and
// visually confirm the robot actually goes forward, not sideways or
// spinning, before trying other directions. If a wheel spins the wrong
// way, the fix is EncoderDriverConfig.direction_inverted in
// config/encoder_config.c (motor wiring) or the corner-sign comment in
// drivetrain_velocity_kinematics.cpp -- not something to paper over here.
//
// Serial commands (newline-terminated):
//   drive <angle_deg> [velocity_mps] [duration_ms]
//                     -- translate at velocity_mps (default: drive_speed_mps)
//                     in that body-frame direction for duration_ms (default:
//                     kDriveDurationMs), then stop. angle_deg=0 is forward
//                     (+vx), 90 is strafe right (+vy), matching
//                     DrivetrainBodyVelocity's convention. Any angle is
//                     accepted, not just the 8 compass points the dashboard
//                     buttons send.
//   turn <angle_deg> [omega_rad_s]
//                     -- rotate in place by (approximately) angle_deg at
//                     omega_rad_s (default: turn_speed_rad_s), then stop.
//                     Positive angle is CCW, negative is CW, matching
//                     DrivetrainBodyVelocity's omega convention. This is
//                     OPEN-LOOP: there's no heading sensor here (no
//                     IMU/odometry wired into this harness), so the
//                     duration is just computed as angle/omega_rad_s and
//                     run blind -- wheel slip/friction mean the actual
//                     angle turned won't be exact. Good enough to test
//                     with, not a real closed-loop position controller.
//   spin <omega_rad_s> <duration_ms>
//                     -- rotate in place at a directly-commanded angular
//                     velocity for duration_ms, then stop. Unlike "turn",
//                     this isn't computed from a target angle -- it's for
//                     watching sustained rotation behavior (the same way
//                     tuning_main.cpp's "continuous" mode holds a target
//                     speed) rather than sweeping a specific angle.
//   speed <mps>        -- set the default velocity used when "drive" is
//                     called without one. Default matches kDefaultDriveSpeedMps.
//   turnspeed <rad_s>  -- set the default omega used when "turn" is called
//                     without one. Default matches kDefaultTurnSpeedRadS.
//   ff/ffo/kp/ki <value>
//                     -- live-tune the shared WheelVelocityPiConfig gains,
//                     same meaning as tuning_main.cpp's commands of the
//                     same name -- lets you watch the effect on all 4
//                     wheels while actually driving, not just one wheel on
//                     the bench. RAM only, same as everything else here;
//                     copy values into wheel_velocity_pi_config.cpp by hand
//                     once you're happy.
//   stop               -- immediately cancel any in-progress drive/turn/spin
//                     and command a hold-at-zero (still closed-loop, not
//                     a raw motor cut -- see wheel_velocity_pi.h's
//                     stop-shouldn't-inherit-integral-windup behavior).
//
// Streams "millis,wheel,target_mps,measured_mps,duty" telemetry (wheel is
// 1=FL/2=BL/3=FR/4=BR, physical connector order) at kPrintPeriodMs regardless of whether a
// drive/turn/spin is active, same rate-limiting rationale and CSV shape as
// tuning_main.cpp -- so tools/drive_dashboard.html can chart it the same
// way tools/tuning_dashboard.html does.
//
// speed/turnspeed/ff/ffo/kp/ki live only in RAM -- they reset to the
// defaults below on reboot, same as every other RAM-only setting in this
// codebase's tuning harnesses.
//
// Also usable from tools/drive_dashboard.html -- same text protocol, sent
// either over Web Serial (USB) or a WebSocket, connected to this board's
// WiFi AP (debug-only setup: open network, no password). The dashboard
// picks the transport; the command text and every reply are identical
// either way (see output_line() below, which writes to both Serial and
// any connected WebSocket clients).

namespace {

constexpr float kDefaultDriveSpeedMps = 0.4f;
constexpr float kDefaultTurnSpeedRadS = 1.0f; // unverified starting guess -- adjust via "turnspeed" once you see how it behaves
constexpr unsigned long kDriveDurationMs = 1000;

constexpr float kPi = 3.14159265358979323846f;

// Debug-only AP: open network (no password) so connecting from a laptop
// during bench testing is one click. Not meant to be internet-facing.
constexpr const char *kApSsid = "esp32-drivetrain";
constexpr uint16_t kWebSocketPort = 81;

WebSocketsServer webSocket(kWebSocketPort);

// Writes one reply line to both Serial and every connected WebSocket
// client, so the dashboard's log sees the same output regardless of which
// transport sent the command.
void output_line(String line) {
    Serial.println(line);
    webSocket.broadcastTXT(line);
}

// Order matches the physical connector numbering (M1/E1..M4/E4 on the
// board), NOT DrivetrainWheelVelocity's fl/fr/bl/br field order -- M1=FL,
// M2=BL, M3=FR, M4=BR (see motor_config.c/encoder_config.c's per-motor
// comments). Wheel indices reported over telemetry (drive_dashboard.html)
// and the target_mps array below both follow this same physical order, so
// "wheel 1" in the UI always means the wheel physically wired to
// connector 1, matching tuning_main.cpp's "motor 1..4" convention.
const MotorDriverConfig *const kMotorConfigs[4] = {
    &FL_MOTOR_CONFIG, &BL_MOTOR_CONFIG, &FR_MOTOR_CONFIG, &BR_MOTOR_CONFIG
};
const EncoderDriverConfig *const kEncoderConfigs[4] = {
    &FL_ENCODER_CONFIG, &BL_ENCODER_CONFIG, &FR_ENCODER_CONFIG, &BR_ENCODER_CONFIG
};

MotorDriver motors[4] = {};
EncoderDriver encoders[4];
WheelVelocityPi pi_states[4];

// Mutable (unlike the old const-ref default) so "ff"/"ffo"/"kp"/"ki"
// commands can live-tune it, same as tuning_main.cpp's pi_config.
WheelVelocityPiConfig pi_config = WHEEL_VELOCITY_PI_CONFIG;

float drive_speed_mps = kDefaultDriveSpeedMps;
float turn_speed_rad_s = kDefaultTurnSpeedRadS;

bool driving = false;
unsigned long drive_end_ms = 0;
float drive_vx = 0.0f;
float drive_vy = 0.0f;
float drive_omega = 0.0f;

unsigned long last_update_ms = 0;

// Telemetry print-rate limiting -- see tuning_main.cpp's kPrintPeriodMs
// comment for why this is separate from the control loop's own rate.
constexpr unsigned long kPrintPeriodMs = 20; // 50 Hz
unsigned long last_print_ms = 0;

// Shared by start_drive()/start_turn() -- one only ever sets vx/vy, the
// other only ever sets omega, but both go through the same
// timer/reset/logging path. duration_ms is per-call rather than a shared
// constant since start_turn() computes it from the requested angle.
void start_body_command(float vx, float vy, float omega, unsigned long duration_ms, const char *label) {
    drive_vx = vx;
    drive_vy = vy;
    drive_omega = omega;
    driving = true;
    drive_end_ms = millis() + duration_ms;

    for (int i = 0; i < 4; i++) wheel_velocity_pi_reset(pi_states[i]);

    output_line("# " + String(label) + " vx=" + String(drive_vx, 4) +
                " vy=" + String(drive_vy, 4) + " omega=" + String(drive_omega, 4) +
                " duration_ms=" + String(duration_ms));
}

void start_drive(float angle_deg, float velocity_mps, unsigned long duration_ms) {
    const float angle_rad = angle_deg * kPi / 180.0f;
    start_body_command(velocity_mps * cosf(angle_rad), velocity_mps * sinf(angle_rad), 0.0f, duration_ms, "driving");
}

void start_turn(float turn_angle_deg, float omega_mag_rad_s) {
    if (omega_mag_rad_s <= 0.0f) {
        output_line("# omega must be > 0 -- pass a value or set it with turnspeed first");
        return;
    }

    // Negated relative to turn_angle_deg's sign: on the real robot,
    // commanding turn_angle_deg > 0 rotated the chassis the opposite way
    // from what the angle's sign implied, so this flips omega alone --
    // translation's forward/reverse sense (motor_config.c's
    // direction_inverted) and the per-wheel FL/FR/BL/BR rotation pairing
    // (motor_config.c/encoder_config.c connector assignment) are both
    // already correct and untouched by this.
    const float omega = (turn_angle_deg < 0.0f) ? omega_mag_rad_s : -omega_mag_rad_s;
    const float turn_angle_rad = fabsf(turn_angle_deg) * kPi / 180.0f;
    const unsigned long duration_ms = (unsigned long)(turn_angle_rad / omega_mag_rad_s * 1000.0f);

    start_body_command(0.0f, 0.0f, omega, duration_ms, "turning");
}

// Directly-commanded sustained rotation, unlike start_turn()'s
// angle-to-duration computation -- the "continuous mode" equivalent for
// rotation.
void start_spin(float omega_rad_s, unsigned long duration_ms) {
    start_body_command(0.0f, 0.0f, omega_rad_s, duration_ms, "spinning");
}

void stop_drive() {
    driving = false;
    drive_vx = 0.0f;
    drive_vy = 0.0f;
    drive_omega = 0.0f;
    output_line("# stopped");
}

void print_usage() {
    output_line("# usage: drive <angle_deg> [velocity_mps] [duration_ms] | turn <angle_deg> [omega_rad_s] | "
                "spin <omega_rad_s> <duration_ms> | speed <mps> | turnspeed <rad_s> | "
                "ff|ffo|kp|ki <value> | stop");
}

// Splits whitespace-separated args (everything after the command key) into
// up to max_tokens Strings. Returns the count actually found.
int split_tokens(String rest, String tokens[], int max_tokens) {
    int count = 0;
    rest.trim();
    while (count < max_tokens && rest.length() > 0) {
        const int sp = rest.indexOf(' ');
        if (sp < 0) {
            tokens[count++] = rest;
            break;
        }
        tokens[count++] = rest.substring(0, sp);
        rest = rest.substring(sp + 1);
        rest.trim();
    }
    return count;
}

// Shared by both transports -- handle_serial_command() feeds it lines read
// from Serial, on_websocket_event() feeds it lines received as WebSocket
// text frames. Same commands, same replies, either way.
void process_command_line(String line) {
    line.trim();
    if (line.length() == 0) return;

    if (line == "stop") {
        stop_drive();
        return;
    }

    const int space = line.indexOf(' ');
    if (space < 0) {
        print_usage();
        return;
    }

    const String key = line.substring(0, space);
    const String rest = line.substring(space + 1);

    String tokens[3];
    const int token_count = split_tokens(rest, tokens, 3);
    if (token_count == 0) {
        print_usage();
        return;
    }

    if (key == "drive") {
        const float angle_deg = tokens[0].toFloat();
        const float velocity_mps = (token_count >= 2) ? tokens[1].toFloat() : drive_speed_mps;
        const unsigned long duration_ms = (token_count >= 3) ? (unsigned long)tokens[2].toFloat() : kDriveDurationMs;
        start_drive(angle_deg, velocity_mps, duration_ms);
    } else if (key == "turn") {
        const float angle_deg = tokens[0].toFloat();
        const float omega_rad_s = (token_count >= 2) ? tokens[1].toFloat() : turn_speed_rad_s;
        start_turn(angle_deg, omega_rad_s);
    } else if (key == "spin") {
        if (token_count < 2) {
            output_line("# usage: spin <omega_rad_s> <duration_ms>");
            return;
        }
        start_spin(tokens[0].toFloat(), (unsigned long)tokens[1].toFloat());
    } else if (key == "speed") {
        drive_speed_mps = tokens[0].toFloat();
        output_line("# drive_speed_mps = " + String(drive_speed_mps, 4));
    } else if (key == "turnspeed") {
        turn_speed_rad_s = tokens[0].toFloat();
        output_line("# turn_speed_rad_s = " + String(turn_speed_rad_s, 4));
    } else if (key == "ff") {
        pi_config.kff = tokens[0].toFloat();
        output_line("# kff = " + String(pi_config.kff, 4));
    } else if (key == "ffo") {
        pi_config.kff_offset = tokens[0].toFloat();
        output_line("# kff_offset = " + String(pi_config.kff_offset, 4));
    } else if (key == "kp") {
        pi_config.kp = tokens[0].toFloat();
        output_line("# kp = " + String(pi_config.kp, 4));
    } else if (key == "ki") {
        pi_config.ki = tokens[0].toFloat();
        output_line("# ki = " + String(pi_config.ki, 4));
    } else {
        print_usage();
    }
}

void handle_serial_command() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    process_command_line(line);
}

void on_websocket_event(uint8_t client_num, WStype_t type, uint8_t *payload, size_t length) {
    if (type != WStype_TEXT) return;
    process_command_line(String((char *)payload, length));
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);

    for (int i = 0; i < 4; i++) {
        encoder_driver_init(&encoders[i], kEncoderConfigs[i]);
        encoder_driver_start(&encoders[i]);

        motor_driver_init(&motors[i], kMotorConfigs[i]);
        motor_driver_enable(&motors[i]);
    }

    WiFi.softAP(kApSsid);
    Serial.print("# WiFi AP \"");
    Serial.print(kApSsid);
    Serial.print("\" up, connect a WebSocket to ws://");
    Serial.print(WiFi.softAPIP());
    Serial.print(":");
    Serial.println(kWebSocketPort);

    webSocket.begin();
    webSocket.onEvent(on_websocket_event);

    Serial.println("# ready.");
    print_usage();
    Serial.println("millis,wheel,target_mps,measured_mps,duty");

    last_update_ms = millis();
}

void loop() {
    webSocket.loop();
    handle_serial_command();

    const unsigned long now = millis();
    const float dt_s = (now - last_update_ms) / 1000.0f;
    if (dt_s <= 0.0f) return;
    last_update_ms = now;

    if (driving && now >= drive_end_ms) {
        stop_drive();
    }

    DrivetrainBodyVelocity body;
    body.vx = driving ? drive_vx : 0.0f;
    body.vy = driving ? drive_vy : 0.0f;
    body.omega = driving ? drive_omega : 0.0f;

    DrivetrainWheelVelocity wheel_rad_s;
    drivetrain_kinematics_body_to_wheel_velocities(DRIVETRAIN_VELOCITY_KINEMATICS_CONFIG, body, wheel_rad_s);

    // rad/s -> m/s via v = r*omega, matching the unit-consistency note in
    // control/wheel_velocity_pi.h. Reordered to physical connector order
    // (fl, bl, fr, br) to zip against kMotorConfigs/kEncoderConfigs above.
    const float r = DRIVETRAIN_VELOCITY_KINEMATICS_CONFIG.wheel_radius_m;
    const float target_mps[4] = {
        wheel_rad_s.fl * r, wheel_rad_s.bl * r, wheel_rad_s.fr * r, wheel_rad_s.br * r
    };

    const bool should_print = (now - last_print_ms >= kPrintPeriodMs);
    if (should_print) last_print_ms = now;

    for (int i = 0; i < 4; i++) {
        encoder_driver_update(&encoders[i]);
        const float measured_mps = encoder_driver_get_velocity_mps(&encoders[i]);

        float duty = 0.0f;
        wheel_velocity_pi_update(pi_states[i], pi_config, target_mps[i], measured_mps, dt_s, duty);
        motor_driver_set_duty(&motors[i], duty);

        if (should_print) {
            output_line(String(now) + "," + String(i + 1) + "," + String(target_mps[i], 4) +
                        "," + String(measured_mps, 4) + "," + String(duty, 4));
        }
    }
}
