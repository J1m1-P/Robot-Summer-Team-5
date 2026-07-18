/* Runs the complete drivetrain through serial or WebSocket debug commands. */
#include <Arduino.h>
#include <math.h>

#include <WiFi.h>
#include <WebSocketsServer.h>
#include <esp_timer.h>

#include "config/drivetrain/drivetrain_config.h"
#include "control/drivetrain/drivetrain.h"

#include <robot_common/app_log.h>

namespace {

constexpr float kDefaultDriveSpeedMps = 0.4f;
constexpr float kDefaultTurnSpeedRadS = 1.0f;
constexpr unsigned long kDriveDurationMs = 1000;
constexpr unsigned long kMaxCommandDurationMs = 60000;
constexpr unsigned long kPrintPeriodMs = 20;
constexpr float kPi = 3.14159265358979323846f;
constexpr const char *kApSsid = "esp32-drivetrain";
constexpr uint16_t kWebSocketPort = 81;

WebSocketsServer web_socket(kWebSocketPort);
Drivetrain drivetrain;
WheelVelocityPiConfig pi_config = DRIVETRAIN_CONFIG.wheel_pi;

// Maps logical FL/FR/BL/BR order to physical connector order M1/M2/M3/M4.
const DrivetrainMotorId kDisplayOrder[DRIVETRAIN_MOTOR_MAX] = {
    DRIVETRAIN_MOTOR_FL,
    DRIVETRAIN_MOTOR_BL,
    DRIVETRAIN_MOTOR_FR,
    DRIVETRAIN_MOTOR_BR,
};

float drive_speed_mps = kDefaultDriveSpeedMps;
float turn_speed_rad_s = kDefaultTurnSpeedRadS;
float drive_vx = 0.0f;
float drive_vy = 0.0f;
float drive_omega = 0.0f;
unsigned long drive_end_ms = 0;
unsigned long last_print_ms = 0;
bool drivetrain_ready = false;
bool driving = false;

// Sends one response to USB serial and every connected dashboard.
void output_line(String line) {
    Serial.println(line);
    web_socket.broadcastTXT(line);
}

// Prints the command syntax accepted by both transports.
void print_usage() {
    output_line("# usage: drive <angle_deg> [velocity_mps] [duration_ms] | "
                "turn <angle_deg> [omega_rad_s] | spin <omega_rad_s> <duration_ms> | "
                "speed <mps> | turnspeed <rad_s> | ff|ffo|kp|ki <value> | stop");
}

// Splits a whitespace-delimited argument list into a bounded token array.
int split_tokens(String rest, String tokens[], int max_tokens) {
    int count = 0;
    rest.trim();
    while (count < max_tokens && rest.length() > 0) {
        const int space = rest.indexOf(' ');
        if (space < 0) {
            tokens[count++] = rest;
            break;
        }
        tokens[count++] = rest.substring(0, space);
        rest = rest.substring(space + 1);
        rest.trim();
    }
    return count;
}

// Parses a positive, bounded duration so malformed commands cannot run forever.
bool parse_duration_ms(const String &token, unsigned long &duration_out) {
    const float value = token.toFloat();
    if (!isfinite(value) || value <= 0.0f || value > kMaxCommandDurationMs) {
        output_line("# duration must be between 1 and " +
                    String(kMaxCommandDurationMs) + " ms");
        return false;
    }
    duration_out = static_cast<unsigned long>(value);
    return true;
}

// Starts a timed body-velocity command through the production controller.
void start_body_command(
    float vx,
    float vy,
    float omega,
    unsigned long duration_ms,
    const char *label
) {
    if (!drivetrain_ready || duration_ms == 0 ||
        duration_ms > kMaxCommandDurationMs) {
        output_line("# drivetrain unavailable or duration is out of range");
        return;
    }
    const esp_err_t error = drivetrain_set_body_velocity(
        &drivetrain, vx, vy, omega);
    if (error != ESP_OK) {
        output_line("# command rejected: " + String(esp_err_to_name(error)));
        return;
    }

    drive_vx = vx;
    drive_vy = vy;
    drive_omega = omega;
    drive_end_ms = millis() + duration_ms;
    driving = true;
    output_line("# " + String(label) + " vx=" + String(vx, 4) +
                " vy=" + String(vy, 4) + " omega=" + String(omega, 4) +
                " duration_ms=" + String(duration_ms));
}

// Converts a compass angle and speed into a timed translation command.
void start_drive(float angle_deg, float velocity_mps, unsigned long duration_ms) {
    const float angle_rad = angle_deg * kPi / 180.0f;
    start_body_command(velocity_mps * cosf(angle_rad),
                       velocity_mps * sinf(angle_rad), 0.0f,
                       duration_ms, "driving");
}

// Converts an approximate turn angle into a timed angular-velocity command.
void start_turn(float angle_deg, float omega_magnitude_rad_s) {
    if (omega_magnitude_rad_s <= 0.0f) {
        output_line("# omega must be greater than zero");
        return;
    }
    const float omega = angle_deg < 0.0f
        ? omega_magnitude_rad_s
        : -omega_magnitude_rad_s;
    const unsigned long duration_ms = static_cast<unsigned long>(
        fabsf(angle_deg) * kPi / 180.0f / omega_magnitude_rad_s * 1000.0f);
    start_body_command(0.0f, 0.0f, omega, duration_ms, "turning");
}

// Cancels timed motion and requests a controlled zero-velocity stop.
void stop_drive() {
    driving = false;
    drive_vx = 0.0f;
    drive_vy = 0.0f;
    drive_omega = 0.0f;
    if (drivetrain_ready) drivetrain_stop(&drivetrain);
    output_line("# stopped");
}

// Installs edited gains and restores the accepted values if validation fails.
bool apply_pi_config() {
    const esp_err_t error = drivetrain_set_wheel_pi_config(
        &drivetrain, &pi_config);
    if (error == ESP_OK) return true;
    drivetrain_get_wheel_pi_config(&drivetrain, &pi_config);
    output_line("# invalid PI configuration rejected");
    return false;
}

// Parses and executes one complete serial or WebSocket command line.
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
    String tokens[3];
    const int count = split_tokens(line.substring(space + 1), tokens, 3);
    if (count == 0) {
        print_usage();
        return;
    }

    if (key == "drive") {
        const float speed = count >= 2 ? tokens[1].toFloat() : drive_speed_mps;
        unsigned long duration = kDriveDurationMs;
        if (count >= 3 && !parse_duration_ms(tokens[2], duration)) return;
        start_drive(tokens[0].toFloat(), speed, duration);
    } else if (key == "turn") {
        const float omega = count >= 2 ? tokens[1].toFloat() : turn_speed_rad_s;
        start_turn(tokens[0].toFloat(), omega);
    } else if (key == "spin" && count >= 2) {
        unsigned long duration = 0;
        if (!parse_duration_ms(tokens[1], duration)) return;
        start_body_command(0.0f, 0.0f, tokens[0].toFloat(), duration, "spinning");
    } else if (key == "speed") {
        drive_speed_mps = tokens[0].toFloat();
        output_line("# drive_speed_mps = " + String(drive_speed_mps, 4));
    } else if (key == "turnspeed") {
        turn_speed_rad_s = tokens[0].toFloat();
        output_line("# turn_speed_rad_s = " + String(turn_speed_rad_s, 4));
    } else if (key == "ff" || key == "ffo" || key == "kp" || key == "ki") {
        const float value = tokens[0].toFloat();
        if (key == "ff") pi_config.kff = value;
        if (key == "ffo") pi_config.kff_offset = value;
        if (key == "kp") pi_config.kp = value;
        if (key == "ki") pi_config.ki = value;
        if (apply_pi_config()) output_line("# " + key + " = " + String(value, 4));
    } else {
        print_usage();
    }
}

// Reads at most one newline-terminated USB serial command per loop.
void handle_serial_command() {
    if (Serial.available()) process_command_line(Serial.readStringUntil('\n'));
}

// Routes WebSocket text frames through the shared command parser.
void on_websocket_event(uint8_t, WStype_t type, uint8_t *payload, size_t length) {
    if (type == WStype_TEXT) process_command_line(String((char *)payload, length));
}

// Emits one telemetry row per wheel in physical connector order.
void print_telemetry(unsigned long now_ms) {
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        const DrivetrainMotorId wheel = kDisplayOrder[index];
        output_line(String(now_ms) + "," + String(index + 1) + "," +
                    String(drivetrain_get_target_velocity_mps(&drivetrain, wheel), 4) + "," +
                    String(drivetrain_get_encoder_velocity_mps(&drivetrain, wheel), 4) + "," +
                    String(drivetrain_get_applied_duty(&drivetrain, wheel), 4));
    }
}

}  // namespace

// Initializes the unified drivetrain plus its debug communication transports.
void setup() {
    Serial.begin(115200);
    delay(1000);
    app_log_init();

    esp_err_t error = drivetrain_init(&drivetrain, &DRIVETRAIN_CONFIG);
    if (error == ESP_OK) error = drivetrain_enable(&drivetrain);
    if (error != ESP_OK) {
        Serial.print("# drivetrain setup failed: ");
        Serial.println(esp_err_to_name(error));
        return;
    }
    drivetrain_ready = true;

    WiFi.softAP(kApSsid);
    web_socket.begin();
    web_socket.onEvent(on_websocket_event);
    output_line("# WiFi AP ready at ws://" + WiFi.softAPIP().toString() +
                ":" + String(kWebSocketPort));
    print_usage();
    output_line("millis,wheel,target_mps,measured_mps,duty");
}

// Services commands, refreshes active targets, updates control, and reports telemetry.
void loop() {
    web_socket.loop();
    handle_serial_command();
    if (!drivetrain_ready) {
        delay(100);
        return;
    }

    const unsigned long now_ms = millis();
    if (driving && static_cast<int32_t>(now_ms - drive_end_ms) >= 0) stop_drive();

    const float vx = driving ? drive_vx : 0.0f;
    const float vy = driving ? drive_vy : 0.0f;
    const float omega = driving ? drive_omega : 0.0f;
    esp_err_t error = drivetrain_set_body_velocity(&drivetrain, vx, vy, omega);
    if (error == ESP_OK) error = drivetrain_update(&drivetrain, esp_timer_get_time());
    if (error != ESP_OK) {
        output_line("# drivetrain update failed: " + String(esp_err_to_name(error)));
        drivetrain_ready = false;
        return;
    }

    if (now_ms - last_print_ms >= kPrintPeriodMs) {
        last_print_ms = now_ms;
        print_telemetry(now_ms);
    }
}
