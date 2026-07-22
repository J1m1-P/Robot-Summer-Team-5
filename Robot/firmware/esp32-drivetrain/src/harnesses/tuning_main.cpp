#include <Arduino.h>

#include <math.h>
#include <string.h>

#include "config/drivetrain/drivetrain_config.h"
#include "control/drivetrain/drivetrain.h"
#include "control/drivetrain/speed_profile.h"
#include "control/drivetrain/wheel_velocity_controller.h"
#include "control/drivetrain/x_drive_kinematics.h"
#include "drivers/encoder/encoder_driver.h"
#include "drivers/motor/motor_driver.h"

// -----------------------------------------------------------------------------
// Real-time tuning only
// This entire file is an isolated RAM-only wheel-controller tuning application.
// -----------------------------------------------------------------------------

// Multi-wheel tuning harness -- see platformio.ini's [env:tuning]. Not the
// production entry point; kept entirely separate from main.cpp.
//
// Any subset of the 4 motor/encoder pairs can be enabled simultaneously
// (see the "motor" command below) -- each enabled one gets its own
// MotorDriver/EncoderDriver/WheelVelocityController instance, but all
// enabled motors share one run configuration, so multiple wheels can be
// compared side by side in one run.
//
// Nothing moves on its own. Every run is manual: set up the mode and its
// setpoint, hit "start", the robot runs for exactly `duration_ms` at that
// setpoint, then the setpoint is zeroed and telemetry keeps streaming for a
// further kPostRunTailMs (so the coast-down/decay after the command ends is
// visible, not just the instant it stops) before the run fully ends and the
// dashboard's chart freezes. Streams one CSV line per enabled motor at
// kPrintPeriodMs (50 Hz) the whole time (including the tail) --
// "millis,motor,target_mps,measured_mps,duty" (motor is 1..4) -- plottable
// with tools/tuning_dashboard.html. In Duty mode the "target_mps" column
// is repurposed to show the commanded duty (signed, see "turn" below), not
// a speed -- there's no target speed in open-loop duty mode.
//
// Two modes (see "mode" below):
//   Duty mode -- open-loop: commands a fixed raw duty directly to the
//     motor(s), bypassing the PI loop entirely. For building the raw
//     duty->speed curve (feedforward characterization) or finding the
//     real achievable top speed/omega at a given duty, including duties
//     that saturate before reaching a requested target in PI mode.
//   PI mode -- closed-loop: commands a fixed target speed through the
//     existing FF+PI wheel_velocity_controller, using whatever ff/ffo/kp/ki
//     are currently set. For step-response / gain tuning.
//
// Every run tracks and prints the peak reading observed once the run's
// duration elapses: peak |measured_mps| per motor (both modes), plus peak
// |omega| if "turn" is on (see below) -- for a held setpoint this is
// effectively the settled value once transients die out.
//
// Serial commands (newline-terminated), typeable any time:
//   show             -- print the current ff/ffo/kp/ki, e.g. after connecting
//                     or reconnecting -- the dashboard's input fields don't
//                     know the board's live values, only what was compiled
//                     in or last set this session, so this is the only way
//                     to confirm what's actually running before you touch
//                     "Set" on a field and possibly overwrite a tuned gain
//                     with a stale/default one.
//   mode duty|pi     -- select open-loop duty mode or closed-loop PI mode.
//                     Stops any active run (switching mode mid-run isn't
//                     meaningful).
//   duty <value>     -- (duty mode) the raw duty to command, e.g. 0.35.
//   target <value>   -- (pi mode) the target speed, m/s.
//   duration <ms>    -- how long a run lasts before auto-stopping.
//   turn <0|1>       -- 0 = straight (all enabled motors get the same
//                     signed setpoint). 1 = in-place rotation: FL/BL get
//                     the setpoint negated, FR/BR get it as-is -- matches
//                     x_drive_kinematics' pure-rotation sign pattern, so
//                     the wheel speeds combine into a body omega instead
//                     of a forward speed. Turning this on force-enables
//                     all four motors on the next "start" (a rotation
//                     reading needs every wheel).
//   ff <value>       -- (pi mode) set feedforward slope gain (kff), shared by all enabled motors
//   ffo <value>      -- (pi mode) set feedforward offset (kff_offset), shared
//   kp <value>       -- (pi mode) set proportional gain, shared
//   ki <value>       -- (pi mode) set integral gain, shared
//   jerk <value>     -- (pi mode) set max_jerk_mps3 for the speed profile
//                     (S-curve), in m/s^3. Ramps the shared target speed
//                     through a jerk-bounded ramp before it reaches the PI
//                     loop, so PI never sees a step change in target -- does
//                     nothing in duty mode, which commands raw duty directly
//                     by design. Set to 0 to disable (no jerk limiting, PI
//                     sees the raw step target).
//   accel <value>    -- (pi mode) set max_accel_mps2, the acceleration
//                     ceiling the speed profile ramps within -- independent
//                     of "jerk", which only bounds how fast *that* ceiling
//                     is approached.
//   invert <0|1>     -- flip the sign of the measured speed reading, shared
//   motor <1..4> <0|1> -- enable/disable one motor/encoder pair:
//                     1=M1/E1 (FL), 2=M2/E2 (BL), 3=M3/E3 (FR), 4=M4/E4 (BR)
//                     -- physical connector order, see motor_config.c's
//                     per-motor comments. Enabling initializes that pair and resets its PI
//                     integral; disabling stops it. Independent of the
//                     other three -- any combination can be on at once.
//   start            -- begins a run at the current mode/setpoint/duration.
//   stop             -- ends an active run immediately and zeroes duty.
//
// invert is a tuning-session convenience for when the encoder reports the
// opposite sign from what the commanded duty direction would suggest --
// it only flips the value fed into this loop and printed over serial, it
// does NOT touch EncoderDriverConfig.direction_inverted (that's
// compile-time, in config/drivetrain/encoder_config.c, and is the real fix once you
// know which way it should go).
//
// All of the above live only in RAM here -- they reset to this file's
// compiled-in defaults on reboot. Once you land on values you like, copy
// the gains into config/drivetrain/drivetrain_config.c by hand; this build never
// writes anything persistent.
//
// To characterize kff specifically: enable one motor at a time, use Duty
// mode, run a few different duties, and read each run's peak_mps -- kff is
// roughly duty / peak_mps once the wheel has settled.
//
// Also usable headlessly from tools/tuning_dashboard.html (Web Serial API
// browser page) for live charting and point-and-click gain entry instead
// of typing raw commands -- same protocol, same commands, that page just
// sends these same lines and parses the same CSV output.

namespace {

constexpr int kMotorCount = 4;

constexpr float kDefaultRunDuty = 0.0f;
constexpr float kDefaultTargetMps = 1.0f;
constexpr unsigned long kDefaultRunDurationMs = 3000;
// Matches MoveSConfig's own placeholder default (move_s.c) -- not yet a
// calibrated slip-avoidance ceiling, just a reasonable starting point.
constexpr float kDefaultMaxAccelMps2 = 0.5f;
// After the commanded duration elapses and duty/target are zeroed, keep
// streaming telemetry (and keep the chart live) for this much longer before
// actually stopping -- long enough to see the coast-down/decay after the
// setpoint drops to zero, instead of the chart freezing at the exact instant
// the command ends.
constexpr unsigned long kPostRunTailMs = 3000;

// The control loop itself runs every iteration of loop() (as fast as the
// ESP32 can go, likely thousands of Hz) -- that's fine, more control
// updates only helps. But printing a CSV line every single iteration
// floods the terminal and leaves no room to type commands, so telemetry
// is rate-limited separately at kPrintPeriodMs. 50 Hz is still <10% of
// the 115200 baud link's throughput at this line length (~30 bytes/line
// -> ~1.5 KB/s of ~11.5 KB/s available) even with a single motor enabled;
// with all 4 enabled the per-tick output is proportionally larger
// (~4x), still comfortably under the link's capacity.
constexpr unsigned long kPrintPeriodMs = 20; // 50 Hz

const MotorDriverConfig *const kMotorConfigs[kMotorCount] = {
    DRIVETRAIN_CONFIG.motor_configs[DRIVETRAIN_MOTOR_FL],
    DRIVETRAIN_CONFIG.motor_configs[DRIVETRAIN_MOTOR_BL],
    DRIVETRAIN_CONFIG.motor_configs[DRIVETRAIN_MOTOR_FR],
    DRIVETRAIN_CONFIG.motor_configs[DRIVETRAIN_MOTOR_BR],
};
const EncoderDriverConfig *const kEncoderConfigs[kMotorCount] = {
    DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_FL],
    DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_BL],
    DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_FR],
    DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_BR],
};
const char *const kMotorNames[kMotorCount] = {
    "M1/E1 (FL)", "M2/E2 (BL)", "M3/E3 (FR)", "M4/E4 (BR)"
};

MotorDriver motors[kMotorCount] = {};
EncoderDriver encoders[kMotorCount];
WheelVelocityController controller_states[kMotorCount];
bool motor_enabled[kMotorCount] = {true, false, false, false}; // M1 enabled by default, matching the old single-motor default

WheelVelocityControllerConfig controller_config = DRIVETRAIN_CONFIG.wheel_controller;

enum class TuningMode { kDuty, kPi };
TuningMode tuning_mode = TuningMode::kPi;

bool turn_mode = false;
float run_duty = kDefaultRunDuty;
float run_target_mps = kDefaultTargetMps;
unsigned long run_duration_ms = kDefaultRunDurationMs;
bool invert_measurement = false;

SpeedProfileConfig speed_profile_config = {.max_jerk_mps3 = 1.0f};
SpeedProfile speed_profile_state = {0.0f, 0.0f};
float run_max_accel_mps2 = kDefaultMaxAccelMps2;

bool running = false;    // true for the whole run + its post-run tail
bool commanding = false; // true only while actively driving the setpoint
unsigned long run_start_ms = 0;
unsigned long tail_start_ms = 0;
float peak_mps[kMotorCount] = {0.0f, 0.0f, 0.0f, 0.0f};
float peak_omega_rad_s = 0.0f;

unsigned long last_update_ms = 0;
unsigned long last_print_ms = 0;

void print_usage() {
    Serial.println("# usage: show | mode duty|pi | duty <value> | target <value> | duration <ms> | turn <0|1> | ff <value> | ffo <value> | kp <value> | ki <value> | jerk <value> | accel <value> | invert <0|1> | motor <1..4> <0|1> | start | stop");
}

// Prints the gains actually in effect right now -- the only source of
// truth once a session has diverged from the compiled-in defaults.
void print_gains() {
    Serial.print("# kff=");
    Serial.print(controller_config.kff, 4);
    Serial.print(" kff_offset=");
    Serial.print(controller_config.kff_offset, 4);
    Serial.print(" kp=");
    Serial.print(controller_config.kp, 4);
    Serial.print(" ki=");
    Serial.println(controller_config.ki, 4);
}

// Enables or disables one motor/encoder pair independently of the other
// three. Enabling initializes that pair's hardware and resets its PI
// integral (old accumulator state doesn't mean anything for a fresh
// start); disabling stops it. Safe to call redundantly (enabling an
// already-enabled motor, or disabling an already-disabled one) --
// motor_driver_disable()/encoder_driver_stop() guard on !initialized and
// the *_init()/_start()/_enable() calls are idempotent re-configurations.
void set_motor_enabled(int index, bool enabled) {
    if (index < 0 || index >= kMotorCount) {
        Serial.println("# motor index must be 1..4");
        return;
    }

    if (enabled) {
        encoder_driver_init(&encoders[index], kEncoderConfigs[index]);
        encoder_driver_start(&encoders[index]);

        motor_driver_init(&motors[index], kMotorConfigs[index]);
        motor_driver_enable(&motors[index]);

        wheel_velocity_controller_reset(&controller_states[index]);
    } else {
        motor_driver_disable(&motors[index]);
        encoder_driver_stop(&encoders[index]);
    }

    motor_enabled[index] = enabled;

    Serial.print("# motor ");
    Serial.print(kMotorNames[index]);
    Serial.println(enabled ? " = enabled" : " = disabled");
}

// Immediately ends an active run (including any post-run tail) and zeroes
// duty on every enabled motor -- commands duty directly rather than going
// back through the PI loop, so the motor actually stops regardless of which
// mode the run was in. This is what the dashboard freezes the chart on.
void stop_run() {
    running = false;
    commanding = false;
    for (int i = 0; i < kMotorCount; i++) {
        if (motor_enabled[i]) motor_driver_set_duty(&motors[i], 0.0f);
    }
    Serial.println("# run stop");
}

// Starts a manual run at the current mode/setpoint/duration. "turn" force-
// enables all four motors first, since a rotation reading needs every
// wheel; otherwise uses whatever motors are already enabled.
void start_run() {
    if (turn_mode) {
        for (int i = 0; i < kMotorCount; i++) set_motor_enabled(i, true);
    }

    bool any_enabled = false;
    for (int i = 0; i < kMotorCount; i++) any_enabled = any_enabled || motor_enabled[i];
    if (!any_enabled) {
        Serial.println("# start: enable at least one motor first");
        return;
    }

    for (int i = 0; i < kMotorCount; i++) {
        if (motor_enabled[i]) wheel_velocity_controller_reset(&controller_states[i]);
    }

    running = true;
    commanding = true;
    run_start_ms = millis();
    memset(peak_mps, 0, sizeof(peak_mps));
    peak_omega_rad_s = 0.0f;
    speed_profile_reset(&speed_profile_state, 0.0f);

    Serial.print("# run start mode=");
    Serial.print(tuning_mode == TuningMode::kDuty ? "duty" : "pi");
    Serial.print(" setpoint=");
    Serial.print(tuning_mode == TuningMode::kDuty ? run_duty : run_target_mps, 4);
    Serial.print(" duration_ms=");
    Serial.print(run_duration_ms);
    Serial.print(" turn=");
    Serial.println(turn_mode ? "1" : "0");
}

void handle_serial_command() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    if (line == "show") {
        print_gains();
        return;
    }
    if (line == "start") {
        start_run();
        return;
    }
    if (line == "stop") {
        stop_run();
        return;
    }

    const int firstSpace = line.indexOf(' ');
    if (firstSpace < 0) {
        print_usage();
        return;
    }

    const String key = line.substring(0, firstSpace);
    String rest = line.substring(firstSpace + 1);
    rest.trim();

    if (key == "motor") {
        const int secondSpace = rest.indexOf(' ');
        if (secondSpace < 0) {
            Serial.println("# usage: motor <1..4> <0|1>");
            return;
        }
        const int index = rest.substring(0, secondSpace).toInt() - 1;
        const float enabledValue = rest.substring(secondSpace + 1).toFloat();
        set_motor_enabled(index, enabledValue != 0.0f);
        return;
    }

    if (key == "mode") {
        if (rest == "duty") {
            tuning_mode = TuningMode::kDuty;
            Serial.println("# mode = duty");
        } else if (rest == "pi") {
            tuning_mode = TuningMode::kPi;
            Serial.println("# mode = pi");
        } else {
            Serial.println("# usage: mode duty|pi");
            return;
        }
        if (running) stop_run(); // switching mode mid-run isn't meaningful
        return;
    }

    const float value = rest.toFloat();

    if (key == "ff") {
        controller_config.kff = value;
        Serial.print("# kff = ");
        Serial.println(controller_config.kff, 4);
    } else if (key == "ffo") {
        controller_config.kff_offset = value;
        Serial.print("# kff_offset = ");
        Serial.println(controller_config.kff_offset, 4);
    } else if (key == "kp") {
        controller_config.kp = value;
        Serial.print("# kp = ");
        Serial.println(controller_config.kp, 4);
    } else if (key == "ki") {
        controller_config.ki = value;
        Serial.print("# ki = ");
        Serial.println(controller_config.ki, 4);
    } else if (key == "duty") {
        run_duty = value;
        Serial.print("# duty = ");
        Serial.println(run_duty, 4);
    } else if (key == "target") {
        run_target_mps = value;
        Serial.print("# target_mps = ");
        Serial.println(run_target_mps, 4);
    } else if (key == "duration") {
        if (value > 0.0f) {
            run_duration_ms = (unsigned long)value;
            Serial.print("# duration_ms = ");
            Serial.println(run_duration_ms);
        } else {
            Serial.println("# duration must be > 0");
        }
    } else if (key == "turn") {
        turn_mode = (value != 0.0f);
        Serial.print("# turn = ");
        Serial.println(turn_mode ? "1" : "0");
    } else if (key == "invert") {
        invert_measurement = (value != 0.0f);
        Serial.print("# invert = ");
        Serial.println(invert_measurement ? "1" : "0");
    } else if (key == "jerk") {
        if (isfinite(value) && value >= 0.0f) {
            speed_profile_config.max_jerk_mps3 = value;
            Serial.print("# jerk = ");
            Serial.println(speed_profile_config.max_jerk_mps3, 4);
        } else {
            Serial.println("# jerk must be >= 0");
        }
    } else if (key == "accel") {
        if (isfinite(value) && value > 0.0f) {
            run_max_accel_mps2 = value;
            Serial.print("# accel = ");
            Serial.println(run_max_accel_mps2, 4);
        } else {
            Serial.println("# accel must be > 0");
        }
    } else {
        print_usage();
    }
}

// Drives one iteration of an active run. While `commanding` (still within
// `duration_ms`): commands the setpoint (raw duty or PI target, depending on
// mode), negated on FL/BL when "turn" is on so the wheels drive a rotation
// instead of a straight line, and tracks peak |measured_mps| per motor plus
// peak |omega| (derived from all four wheel speeds via the same Jacobian
// x_drive_kinematics uses elsewhere) when turning. Once `duration_ms`
// elapses, zeroes every command, prints the peak summary, and enters a
// `kPostRunTailMs` tail: telemetry keeps streaming (so the coast-down/decay
// is visible) but nothing is actively driven. `stop_run()` -- and the
// dashboard freezing on it -- only happens once the tail itself elapses (or
// on an explicit "stop").
void service_run(unsigned long now, float dt_s, bool should_print) {
    if (commanding && now - run_start_ms >= run_duration_ms) {
        for (int i = 0; i < kMotorCount; i++) {
            if (motor_enabled[i]) motor_driver_set_duty(&motors[i], 0.0f);
        }
        for (int i = 0; i < kMotorCount; i++) {
            if (!motor_enabled[i]) continue;
            Serial.print("# RUN mode=");
            Serial.print(tuning_mode == TuningMode::kDuty ? "duty" : "pi");
            Serial.print(" motor=");
            Serial.print(kMotorNames[i]);
            Serial.print(" peak_mps=");
            Serial.println(peak_mps[i], 4);
        }
        if (turn_mode) {
            Serial.print("# RUN peak_omega_rad_s=");
            Serial.println(peak_omega_rad_s, 4);
        }
        commanding = false;
        tail_start_ms = now;
    } else if (!commanding && now - tail_start_ms >= kPostRunTailMs) {
        stop_run();
        return;
    }

    // In PI mode, ramp the shared (unsigned) target speed through the speed
    // profile once per cycle -- not per motor -- so every enabled motor sees
    // the same smoothed target instead of only whichever one happens to be
    // index 0. Turn's per-side sign flip is applied after ramping, below,
    // so both sides ramp in lockstep and only differ in sign. During the
    // tail, `commanding` is false, so this is skipped -- motors stay at the
    // zero duty already commanded above and this cycle just reads telemetry.
    float ramped_target_mps = run_target_mps;
    if (commanding && tuning_mode == TuningMode::kPi) {
        float ramped = ramped_target_mps;
        if (speed_profile_update(&speed_profile_state, &speed_profile_config,
                run_target_mps, run_max_accel_mps2, dt_s, &ramped) == ESP_OK) {
            ramped_target_mps = ramped;
        }
        // On ESP_ERR_INVALID_ARG (e.g. jerk == 0, disabling the profile),
        // ramped_target_mps just falls back to the raw target unramped.
    }

    // Local motor index order is FL,BL,FR,BR (see kMotorConfigs above).
    XDriveWheelVelocity wheel_mps = {0.0f, 0.0f, 0.0f, 0.0f};
    for (int i = 0; i < kMotorCount; i++) {
        if (!motor_enabled[i]) continue;

        encoder_driver_update(&encoders[i]);
        float measured_mps = encoder_driver_get_velocity_mps(&encoders[i]);
        if (invert_measurement) measured_mps = -measured_mps;

        // FL/BL spin one way, FR/BR the other -- matches the sign pattern
        // x_drive_kinematics_body_to_wheel_velocities produces for a
        // pure-omega body velocity, so this drives an in-place rotation.
        const bool negate = turn_mode && (i == 0 || i == 1);

        float duty = 0.0f;
        float commanded_display = 0.0f;
        if (!commanding) {
            // Tail: nothing is being driven, motor already coasting from the
            // zero-duty command issued when the run ended -- just observe.
        } else if (tuning_mode == TuningMode::kDuty) {
            duty = negate ? -run_duty : run_duty;
            commanded_display = duty;
            motor_driver_set_duty(&motors[i], duty);
        } else {
            const float target = negate ? -ramped_target_mps : ramped_target_mps;
            commanded_display = target;
            wheel_velocity_controller_update(
                &controller_states[i], &controller_config, target, measured_mps, dt_s, &duty);
            motor_driver_set_duty(&motors[i], duty);
        }

        if (commanding) peak_mps[i] = fmaxf(peak_mps[i], fabsf(measured_mps));

        if (commanding && turn_mode) {
            switch (i) {
                case 0: wheel_mps.fl = measured_mps; break;
                case 1: wheel_mps.bl = measured_mps; break;
                case 2: wheel_mps.fr = measured_mps; break;
                case 3: wheel_mps.br = measured_mps; break;
            }
        }

        if (should_print) {
            Serial.print(now);
            Serial.print(',');
            Serial.print(i + 1);
            Serial.print(',');
            Serial.print(commanded_display, 4);
            Serial.print(',');
            Serial.print(measured_mps, 4);
            Serial.print(',');
            Serial.println(duty, 4);
        }
    }

    if (commanding && turn_mode) {
        const float wheel_radius_m = DRIVETRAIN_CONFIG.x_drive_kinematics.wheel_radius_m;
        const XDriveWheelVelocity wheel_rad_s = {
            wheel_mps.fl / wheel_radius_m,
            wheel_mps.fr / wheel_radius_m,
            wheel_mps.bl / wheel_radius_m,
            wheel_mps.br / wheel_radius_m,
        };
        DrivetrainBodyVelocity body = {0.0f, 0.0f, 0.0f};
        if (x_drive_kinematics_wheel_to_body_velocities(
                &DRIVETRAIN_CONFIG.x_drive_kinematics, &wheel_rad_s, &body) == ESP_OK) {
            peak_omega_rad_s = fmaxf(peak_omega_rad_s, fabsf(body.omega));
        }
    }
}

}  // namespace

// Initializes serial telemetry and the default front-left tuning channel.
void setup() {
    Serial.begin(115200);
    delay(1000);

    set_motor_enabled(0, true); // default to M1/E1 (FL) only
    print_gains(); // so a freshly connected dashboard/terminal shows the
                    // real compiled-in values instead of blank/stale fields

    Serial.println("millis,motor,target_mps,measured_mps,duty");

    const unsigned long now = millis();
    last_update_ms = now;
}

// Services commands and, if a run is active, advances it.
void loop() {
    handle_serial_command();

    const unsigned long now = millis();
    const float dt_s = (now - last_update_ms) / 1000.0f;
    if (dt_s <= 0.0f) return;
    last_update_ms = now;

    if (!running) return;

    const bool should_print = (now - last_print_ms >= kPrintPeriodMs);
    if (should_print) last_print_ms = now;

    service_run(now, dt_s, should_print);
}
