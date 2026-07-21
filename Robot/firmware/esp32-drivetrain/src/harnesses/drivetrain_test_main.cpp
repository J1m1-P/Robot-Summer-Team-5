/* Interactive motor/encoder and complete-drivetrain hardware test harness. */
#include <Arduino.h>

#include <math.h>
#include <esp_timer.h>

#include "config/drivetrain/drivetrain_config.h"
#include "config/tape_following/tape_following_config.h"
#include "control/drivetrain/drivetrain.h"
#include "control/tape_following/tape_follower.h"
#include "control/tape_following/tape_following_controller.h"
#include "sensing/tape_following/tape_line_estimator.h"

namespace {

constexpr float kDefaultPairDuty = 0.25f;
constexpr float kMaxPairDuty = 0.80f;
constexpr float kDefaultMoveSpeedMps = 0.20f;
constexpr float kDefaultTurnSpeedRadS = 1.0f;
constexpr uint32_t kDefaultDurationMs = 10000;
constexpr uint32_t kMaxDurationMs = 10000;
constexpr uint32_t kMaxTapeDurationMs = 60000;
constexpr uint32_t kDefaultMotionTimeoutMs = 10000;
constexpr uint32_t kMaxMotionTimeoutMs = 60000;
constexpr uint32_t kControlPeriodUs = 5000;
constexpr uint32_t kTelemetryPeriodMs = 100;
constexpr uint32_t kTapeSamplePeriodMs = 5;
constexpr float kDefaultTapeCenterMaxMps = 0.20f;
constexpr float kDefaultTapeFollowSpeedMps = 0.30f;
constexpr uint32_t kSequencePauseMs = 750;
constexpr float kDefaultAcceleration = 0.50f;
constexpr float kDefaultDeceleration = 0.80f;
constexpr float kDefaultDistanceToleranceM = 0.01f;
constexpr float kDefaultAngleToleranceDeg = 2.0f;
constexpr float kStoppedVelocityMps = 0.015f;
constexpr float kPi = 3.14159265358979323846f;
constexpr float kInverseSqrtTwo = 0.70710678118f;
constexpr unsigned int kMaxCommandLength = 96;

enum class TestMode {
    IDLE,
    PAIR,
    BODY,
    SEQUENCE,
    DISTANCE,
    ANGLE,
    TAPE_CENTER,
    STOPPING,
};

struct SequenceStep {
    const char *name;
    float vx_factor;
    float vy_factor;
    float omega_factor;
};

const SequenceStep kSequence[] = {
    {"forward",        1.0f,             0.0f,             0.0f},
    {"backward",      -1.0f,             0.0f,             0.0f},
    {"left",           0.0f,            -1.0f,             0.0f},
    {"right",          0.0f,             1.0f,             0.0f},
    {"forward-left",   kInverseSqrtTwo, -kInverseSqrtTwo,  0.0f},
    {"forward-right",  kInverseSqrtTwo,  kInverseSqrtTwo,  0.0f},
    {"backward-left", -kInverseSqrtTwo, -kInverseSqrtTwo,  0.0f},
    {"backward-right",-kInverseSqrtTwo,  kInverseSqrtTwo,  0.0f},
    {"cw",              0.0f,             0.0f,            -1.0f},
    {"ccw",             0.0f,             0.0f,             1.0f},
};

constexpr size_t kSequenceLength = sizeof(kSequence) / sizeof(kSequence[0]);

const char *const kWheelNames[DRIVETRAIN_MOTOR_MAX] = {
    "FL (M1/E1)", "FR (M3/E3)", "BL (M2/E2)", "BR (M4/E4)"
};

Drivetrain drivetrain = {};
DrivetrainConfig test_config = {};
MotorDriverConfig test_motor_configs[DRIVETRAIN_MOTOR_MAX] = {};
EncoderDriverConfig test_encoder_configs[DRIVETRAIN_MOTOR_MAX] = {};

TestMode mode = TestMode::IDLE;
bool drivetrain_ready = false;
bool sequence_step_active = false;
int pair_index = -1;
size_t sequence_index = 0;
float command_vx = 0.0f;
float command_vy = 0.0f;
float command_omega = 0.0f;
float sequence_speed_mps = kDefaultMoveSpeedMps;
float sequence_turn_rad_s = kDefaultTurnSpeedRadS;
uint32_t sequence_step_ms = kDefaultDurationMs;
uint32_t motion_end_ms = 0;
uint32_t next_sequence_ms = 0;
uint32_t last_telemetry_ms = 0;
uint32_t last_tape_sample_ms = 0;
int64_t last_control_us = 0;
String serial_command_line;
uint32_t telemetry_period_ms = kTelemetryPeriodMs;
uint32_t motion_timeout_ms = kDefaultMotionTimeoutMs;
float acceleration_per_s2 = kDefaultAcceleration;
float deceleration_per_s2 = kDefaultDeceleration;
float distance_tolerance_m = kDefaultDistanceToleranceM;
float angle_tolerance_rad = kDefaultAngleToleranceDeg * kPi / 180.0f;
float applied_vx = 0.0f;
float applied_vy = 0.0f;
float applied_omega = 0.0f;
int32_t motion_start_counts[DRIVETRAIN_MOTOR_MAX] = {};
float motion_direction_x = 0.0f;
float motion_direction_y = 0.0f;
float motion_target = 0.0f;
float motion_progress = 0.0f;
float motion_max_speed = 0.0f;
int motion_turn_sign = 1;
uint32_t position_timeout_at_ms = 0;
WheelVelocityControllerConfig live_controller_configs[DRIVETRAIN_MOTOR_MAX] = {};

TapeSensorMux tape_sensor_mux = {};
TapeSensor tape_sensors[TAPE_SENSOR_MODULE_COUNT] = {};
TapeSensor *tape_sensor_list[TAPE_SENSOR_MODULE_COUNT] = {
    &tape_sensors[0], &tape_sensors[1], &tape_sensors[2]
};
uint8_t tape_bits[TAPE_SENSOR_MODULE_COUNT] = {};
bool tape_sensors_ready = false;
int tape_center_sensor_index = 0;
float tape_center_max_mps = kDefaultTapeCenterMaxMps;
float tape_follow_velocity_mps = 0.0f;
float tape_center_polarity = 1.0f;
float tape_center_error = 0.0f;
float tape_center_correction_mps = 0.0f;
bool tape_center_line_present = false;
TapeLineEstimatorState tape_center_estimator_state = {};
TapeFollowingControllerState tape_center_controller_state = {};
TapeFollower tape_follower = {};
TapeFollowerConfig tape_follower_config = {};
TapeFollowerConfig live_tape_config = {};
TapeLineEstimatorConfig tape_front_estimator_config = {};
TapeLineEstimatorConfig tape_back_estimator_config = {};

// Real-time tuning implementations are grouped at the bottom of this namespace.
void print_controller_config();
void print_tape_tuning();
void set_inversion(const String &kind, int wheel, const String *value);
void update_pi_parameter(int wheel, const String &field, float value);
void reset_controller_config(int wheel = -1);
void update_tape_parameter(const String &field, float value);
void reset_tape_tuning();
void print_runtime_settings();
void print_config();
void update_ramp_tuning(float acceleration, float deceleration);
void update_distance_tolerance_tuning(float value_m);
void update_angle_tolerance_tuning(float value_deg);
void update_telemetry_period_tuning(long value_ms);
void update_motion_timeout_tuning(long value_ms);

bool time_reached(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

void print_help() {
    Serial.println("\n# Drivetrain hardware test commands (newline terminated)");
    Serial.println("#   pair <wheel> <duty> [ms]  - one motor/encoder; wheel=fl|fr|bl|br|1..4");
    Serial.println("#   forward|backward|left|right [m/s] [ms]");
    Serial.println("#   forward-left|forward-right|backward-left|backward-right [m/s] [ms]");
    Serial.println("#   cw|ccw [rad/s] [ms]       - timed rotation (positive magnitude)");
    Serial.println("#   distance <direction> <meters> [max_mps] - encoder-relative translation");
    Serial.println("#   turn-angle cw|ccw <degrees> [max_rad_s] - encoder-relative rotation");
    Serial.println("#   sequence [m/s] [ms_each]  - all 8 translations, then CW and CCW");
    Serial.println("#   tape | tape-center front|back [max_mps] [ms] [polarity]");
    Serial.println("#   tape-follow front|back [travel_mps] [max_strafe_mps] [ms] [polarity]");
    Serial.println("#   invert motor|encoder <wheel> [0|1] - RAM-only inversion (omit value to toggle)");
    Serial.println("#   pi show|reset [wheel] | pi <wheel|all> kff|offset|kp|ki|slew <value>");
    Serial.println("#   tape-tune show|reset | tape-tune <field> <value>");
    Serial.println("#   ramp <accel> <decel> | distance-tolerance <m> | angle-tolerance <deg>");
    Serial.println("#   timeout <ms> | telemetry <ms> | reset-encoders | limits");
    Serial.println("#   stop (controlled) | coast | brake | enable | config | status | help");
    Serial.println("# Examples: distance forward 1.0 0.3 | turn-angle ccw 90 0.5 | pi kp 0.4");
    Serial.println("# Lift the robot before testing. All commands stop automatically.\n");
}

int parse_wheel(String token) {
    token.toLowerCase();
    if (token == "fl" || token == "1") return DRIVETRAIN_MOTOR_FL;
    if (token == "fr" || token == "3") return DRIVETRAIN_MOTOR_FR;
    if (token == "bl" || token == "2") return DRIVETRAIN_MOTOR_BL;
    if (token == "br" || token == "4") return DRIVETRAIN_MOTOR_BR;
    return -1;
}

int split_tokens(String line, String tokens[], int capacity) {
    line.trim();
    int count = 0;
    while (line.length() > 0 && count < capacity) {
        const int space = line.indexOf(' ');
        if (space < 0) {
            tokens[count++] = line;
            break;
        }
        tokens[count++] = line.substring(0, space);
        line = line.substring(space + 1);
        line.trim();
    }
    return count;
}

bool parse_duration(const String &token, uint32_t &duration_out,
                    uint32_t maximum_ms = kMaxDurationMs) {
    const long value = token.toInt();
    if (value <= 0 || value > static_cast<long>(maximum_ms)) {
        Serial.printf("# duration must be 1..%lu ms\n",
                      static_cast<unsigned long>(maximum_ms));
        return false;
    }
    duration_out = static_cast<uint32_t>(value);
    return true;
}

void report_error(const char *operation, esp_err_t error) {
    Serial.printf("# ERROR: %s failed: %s\n", operation, esp_err_to_name(error));
    if (drivetrain.status.initialized) drivetrain_brake(&drivetrain);
    drivetrain_ready = false;
    mode = TestMode::IDLE;
}

esp_err_t coast_all() {
    return drivetrain_coast(&drivetrain);
}

bool mode_uses_closed_loop(TestMode current_mode) {
    return current_mode == TestMode::BODY || current_mode == TestMode::DISTANCE ||
           current_mode == TestMode::ANGLE ||
           current_mode == TestMode::TAPE_CENTER ||
           current_mode == TestMode::STOPPING ||
           (current_mode == TestMode::SEQUENCE && sequence_step_active);
}

float move_toward(float current, float target, float maximum_change) {
    if (target > current + maximum_change) return current + maximum_change;
    if (target < current - maximum_change) return current - maximum_change;
    return target;
}

// Slews translation as one vector and rotation as one scalar. The same ramp
// values are used as m/s^2 for translation and rad/s^2 for rotation.
void update_ramped_command(float dt_s) {
    const float current_speed = hypotf(applied_vx, applied_vy);
    const float target_speed = hypotf(command_vx, command_vy);
    const float rate = target_speed > current_speed
        ? acceleration_per_s2 : deceleration_per_s2;
    const float maximum_change = rate * dt_s;
    const float delta_x = command_vx - applied_vx;
    const float delta_y = command_vy - applied_vy;
    const float delta_magnitude = hypotf(delta_x, delta_y);
    if (delta_magnitude <= maximum_change || delta_magnitude <= 1.0e-6f) {
        applied_vx = command_vx;
        applied_vy = command_vy;
    } else {
        applied_vx += delta_x * maximum_change / delta_magnitude;
        applied_vy += delta_y * maximum_change / delta_magnitude;
    }

    if (mode == TestMode::TAPE_CENTER && tape_follow_velocity_mps != 0.0f) {
        /* TapeFollower already acceleration-limits omega. Applying the generic
         * ramp again made real steering too slow to become visible. */
        applied_omega = command_omega;
    } else {
        const float angular_rate = fabsf(command_omega) > fabsf(applied_omega)
            ? acceleration_per_s2 : deceleration_per_s2;
        applied_omega = move_toward(
            applied_omega, command_omega, angular_rate * dt_s);
    }
}

void capture_motion_start_counts() {
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        motion_start_counts[i] = drivetrain_get_encoder_accumulated_count(
            &drivetrain, static_cast<DrivetrainMotorId>(i));
    }
    motion_progress = 0.0f;
}

esp_err_t get_relative_body_motion(DrivetrainBodyVelocity *motion_out) {
    if (motion_out == nullptr) return ESP_ERR_INVALID_ARG;
    float wheel_angle[DRIVETRAIN_MOTOR_MAX] = {};
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        const int32_t count = drivetrain_get_encoder_accumulated_count(
            &drivetrain, static_cast<DrivetrainMotorId>(i));
        const EncoderDriverConfig *config = test_config.encoder_configs[i];
        wheel_angle[i] = static_cast<float>(count - motion_start_counts[i]) *
                         (2.0f * kPi) /
                         static_cast<float>(config->counts_per_revolution);
    }
    const XDriveWheelVelocity wheels = {
        .fl = wheel_angle[DRIVETRAIN_MOTOR_FL],
        .fr = wheel_angle[DRIVETRAIN_MOTOR_FR],
        .bl = wheel_angle[DRIVETRAIN_MOTOR_BL],
        .br = wheel_angle[DRIVETRAIN_MOTOR_BR],
    };
    return x_drive_kinematics_wheel_to_body_velocities(
        &test_config.x_drive_kinematics, &wheels, motion_out);
}

bool direction_vector(const String &name, float *x_out, float *y_out) {
    if (x_out == nullptr || y_out == nullptr) return false;
    *x_out = 0.0f;
    *y_out = 0.0f;
    if (name == "forward") *x_out = 1.0f;
    else if (name == "backward") *x_out = -1.0f;
    else if (name == "left") *y_out = -1.0f;
    else if (name == "right") *y_out = 1.0f;
    else if (name == "forward-left") { *x_out = kInverseSqrtTwo; *y_out = -kInverseSqrtTwo; }
    else if (name == "forward-right") { *x_out = kInverseSqrtTwo; *y_out = kInverseSqrtTwo; }
    else if (name == "backward-left") { *x_out = -kInverseSqrtTwo; *y_out = -kInverseSqrtTwo; }
    else if (name == "backward-right") { *x_out = -kInverseSqrtTwo; *y_out = kInverseSqrtTwo; }
    else return false;
    return true;
}

void begin_controlled_stop(const char *reason) {
    if (!drivetrain_ready) return;
    command_vx = command_vy = command_omega = 0.0f;
    mode = TestMode::STOPPING;
    sequence_step_active = false;
    motion_end_ms = millis() + 2000;
    Serial.printf("# controlled stop: %s\n", reason);
}

void reset_tape_center_state() {
    tape_line_estimator_reset(&tape_center_estimator_state);
    tape_following_controller_reset(&tape_center_controller_state);
    tape_center_error = 0.0f;
    tape_center_correction_mps = 0.0f;
    tape_center_line_present = false;
}

bool wheels_are_stopped() {
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        if (fabsf(drivetrain_get_encoder_velocity_mps(
                &drivetrain, static_cast<DrivetrainMotorId>(i))) >
            kStoppedVelocityMps) {
            return false;
        }
    }
    return hypotf(applied_vx, applied_vy) <= 1.0e-3f &&
           fabsf(applied_omega) <= 1.0e-3f;
}

void stop_test(bool announce = true) {
    if (drivetrain_ready) {
        const esp_err_t error = coast_all();
        if (error != ESP_OK) {
            report_error("coast", error);
            return;
        }
    }
    mode = TestMode::IDLE;
    sequence_step_active = false;
    pair_index = -1;
    command_vx = command_vy = command_omega = 0.0f;
    applied_vx = applied_vy = applied_omega = 0.0f;
    reset_tape_center_state();
    if (announce) Serial.println("# stopped; all motors coasting");
}

void brake_test() {
    if (!drivetrain.status.initialized) {
        Serial.println("# drivetrain is not initialized");
        return;
    }
    const esp_err_t error = drivetrain_brake(&drivetrain);
    mode = TestMode::IDLE;
    drivetrain_ready = false;
    command_vx = command_vy = command_omega = 0.0f;
    applied_vx = applied_vy = applied_omega = 0.0f;
    if (error != ESP_OK) report_error("brake", error);
    else Serial.println("# hardware brake engaged; use enable before motion");
}

void enable_test() {
    if (!drivetrain.status.initialized) {
        Serial.println("# drivetrain is not initialized");
        return;
    }
    if (drivetrain_ready) {
        Serial.println("# drivetrain already enabled");
        return;
    }
    const esp_err_t error = drivetrain_enable(&drivetrain);
    if (error != ESP_OK) {
        Serial.printf("# enable failed: %s\n", esp_err_to_name(error));
        return;
    }
    drivetrain_ready = true;
    last_control_us = esp_timer_get_time();
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        drivetrain_get_motor_controller_config(
            &drivetrain, static_cast<DrivetrainMotorId>(i), &live_controller_configs[i]);
    }
    Serial.println("# drivetrain enabled; motors coasting");
}

void reset_all_encoders() {
    if (!drivetrain.status.initialized) {
        Serial.println("# drivetrain is not initialized");
        return;
    }
    if (drivetrain_ready) stop_test(false);
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        const esp_err_t error = encoder_driver_reset(&drivetrain.devices.encoders[i]);
        if (error != ESP_OK) {
            report_error("encoder reset", error);
            return;
        }
    }
    capture_motion_start_counts();
    Serial.println("# all encoder counts reset to zero");
}

void format_tape_bits(uint8_t bits, bool reverse_channels, char output[5]) {
    for (int channel = 0; channel < TAPE_SENSOR_CHANNEL_COUNT; ++channel) {
        const int source_channel = reverse_channels
            ? TAPE_SENSOR_CHANNEL_COUNT - 1 - channel
            : channel;
        output[channel] = (bits & (1U << source_channel)) ? '1' : '0';
    }
    output[TAPE_SENSOR_CHANNEL_COUNT] = '\0';
}

bool sample_tape_sensors() {
    if (!tape_sensors_ready) return false;
    const esp_err_t error = tape_sensor_driver_read_all_raw(
        tape_sensor_list, tape_bits);
    if (error != ESP_OK) {
        Serial.printf("# ERROR: tape sampling failed: %s\n",
                      esp_err_to_name(error));
        tape_sensors_ready = false;
        if (mode == TestMode::TAPE_CENTER && drivetrain_ready) {
            begin_controlled_stop("tape sensor fault");
        }
        return false;
    }
    return true;
}

void print_tape_telemetry(uint32_t now_ms) {
    char front[5];
    char back[5];
    char left[5];
    /* The front module is mounted opposite the mux channel order. Keep all
     * operator-facing telemetry in physical left-to-right order. */
    format_tape_bits(tape_bits[0], true, front);
    format_tape_bits(tape_bits[1], false, back);
    format_tape_bits(tape_bits[2], false, left);
    const bool target_match = tape_sensors_ready &&
        mode == TestMode::TAPE_CENTER &&
        tape_bits[tape_center_sensor_index] == 0x06;
    const float reported_error = mode == TestMode::TAPE_CENTER
        ? tape_center_error : 0.0f;
    const float reported_correction = mode == TestMode::TAPE_CENTER
        ? tape_center_correction_mps : 0.0f;
    Serial.printf("TAPE,%lu,%s,%s,%s,%s,%.3f,%.3f,%d\n",
                  static_cast<unsigned long>(now_ms), front, back, left,
                  !tape_sensors_ready ? "unavailable" :
                  (mode == TestMode::TAPE_CENTER
                       ? (tape_follow_velocity_mps == 0.0f ? "centering" : "following")
                       : "monitor"),
                  reported_error, reported_correction,
                  target_match ? 1 : 0);
    if (mode == TestMode::TAPE_CENTER && tape_follow_velocity_mps != 0.0f) {
        Serial.printf("# tape-turn,target=%.3f_rad_s,applied=%.3f_rad_s\n",
                      command_omega, applied_omega);
    }
}

void print_telemetry(uint32_t now_ms) {
    Serial.println("# ms,wheel,count,target_mps,measured_mps,duty");
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        const auto wheel = static_cast<DrivetrainMotorId>(i);
        const float target = mode_uses_closed_loop(mode)
            ? drivetrain_get_target_velocity_mps(&drivetrain, wheel) : 0.0f;
        Serial.printf("%lu,%s,%ld,%.4f,%.4f,%.3f\n",
                      static_cast<unsigned long>(now_ms), kWheelNames[i],
                      static_cast<long>(drivetrain_get_encoder_accumulated_count(&drivetrain, wheel)),
                      target,
                      drivetrain_get_encoder_velocity_mps(&drivetrain, wheel),
                      motor_driver_get_current_duty(&drivetrain.devices.motors[i]));
    }
    if (mode == TestMode::DISTANCE) {
        Serial.printf("# motion,distance,progress=%.4f_m,target=%.4f_m,remaining=%.4f_m\n",
                      motion_progress, motion_target,
                      fmaxf(0.0f, motion_target - motion_progress));
    } else if (mode == TestMode::ANGLE) {
        Serial.printf("# motion,angle,progress=%.2f_deg,target=%.2f_deg,remaining=%.2f_deg\n",
                      motion_progress * 180.0f / kPi,
                      motion_target * 180.0f / kPi,
                      fmaxf(0.0f, motion_target - motion_progress) * 180.0f / kPi);
    }
    print_tape_telemetry(now_ms);
}

void start_tape_center(int sensor_index, float maximum_strafe_mps,
                       uint32_t duration_ms, float polarity,
                       float travel_velocity_mps = 0.0f) {
    if (!tape_sensors_ready) {
        Serial.println("# tape sensors are not ready");
        return;
    }
    const esp_err_t error = coast_all();
    if (error != ESP_OK) {
        report_error("tape center start", error);
        return;
    }
    tape_center_sensor_index = sensor_index;
    tape_center_max_mps = maximum_strafe_mps;
    tape_center_polarity = polarity;
    tape_follow_velocity_mps = travel_velocity_mps;
    if (travel_velocity_mps != 0.0f) {
        tape_front_estimator_config = FRONT_TAPE_LINE_ESTIMATOR_CONFIG;
        tape_back_estimator_config = BACK_TAPE_LINE_ESTIMATOR_CONFIG;
        for (int channel = 0; channel < TAPE_SENSOR_CHANNEL_COUNT; ++channel) {
            tape_front_estimator_config.channel_weights[channel] *= polarity;
            tape_back_estimator_config.channel_weights[channel] *= polarity;
        }
        tape_follower_config = live_tape_config;
        tape_follower_config.estimators[TAPE_FOLLOWER_FRONT] =
            &tape_front_estimator_config;
        tape_follower_config.estimators[TAPE_FOLLOWER_BACK] =
            &tape_back_estimator_config;
        tape_follower_config.controller.correction_min = -maximum_strafe_mps;
        tape_follower_config.controller.correction_max = maximum_strafe_mps;
        tape_follower = {};
        const esp_err_t follower_error = tape_follower_init(
            &tape_follower, &tape_follower_config);
        if (follower_error != ESP_OK) {
            report_error("tape follower init", follower_error);
            return;
        }
    }
    reset_tape_center_state();
    command_vx = command_vy = command_omega = 0.0f;
    applied_vx = applied_vy = applied_omega = 0.0f;
    motion_end_ms = millis() + duration_ms;
    mode = TestMode::TAPE_CENTER;
    Serial.printf("# START tape-%s %s: target=0110 travel=%+.3f m/s "
                  "max_strafe=%.3f m/s polarity=%+.0f for %lu ms\n",
                  travel_velocity_mps == 0.0f ? "center" : "follow",
                  sensor_index == 0 ? "front" : "back",
                  travel_velocity_mps, maximum_strafe_mps, polarity,
                  static_cast<unsigned long>(duration_ms));
}

void service_tape_center(float dt_s) {
    if (tape_follow_velocity_mps != 0.0f) {
        const TapeFollowerInput input = {{
            &tape_sensors[0], &tape_sensors[1]}, tape_follow_velocity_mps};
        TapeFollowerOutput output = {};
        const esp_err_t error = tape_follower_update(
            &tape_follower, &input, dt_s, &output);
        if (error != ESP_OK || !output.motion_valid) {
            command_vx = command_vy = command_omega = 0.0f;
            applied_vx = applied_vy = applied_omega = 0.0f;
            tape_center_line_present = output.status == TAPE_FOLLOWER_TRACKING;
            tape_center_error = output.line_error;
            tape_center_correction_mps = 0.0f;
            return;
        }
        tape_center_line_present = output.status == TAPE_FOLLOWER_TRACKING;
        tape_center_error = output.line_error;
        tape_center_correction_mps = output.requested_velocity.vy;
        command_vx = output.requested_velocity.vx;
        command_vy = output.requested_velocity.vy;
        command_omega = output.requested_velocity.omega;
        return;
    }

    const TapeSensor *sensor = &tape_sensors[tape_center_sensor_index];
    const TapeLineEstimatorConfig *estimator = tape_center_sensor_index == 0
        ? &FRONT_TAPE_LINE_ESTIMATOR_CONFIG
        : &BACK_TAPE_LINE_ESTIMATOR_CONFIG;
    tape_center_line_present = tape_line_estimator_compute_error(
        sensor, estimator, &tape_center_estimator_state, &tape_center_error);

    command_vx = 0.0f;
    command_omega = 0.0f;
    if (!tape_center_line_present) {
        tape_following_controller_reset(&tape_center_controller_state);
        tape_center_correction_mps = 0.0f;
        command_vy = 0.0f;
        applied_vx = applied_vy = applied_omega = 0.0f;
        return;
    }

    TapeFollowingControllerConfig controller = live_tape_config.controller;
    controller.correction_min = -tape_center_max_mps;
    controller.correction_max = tape_center_max_mps;
    tape_center_correction_mps = tape_center_polarity *
        tape_following_controller_update(
            &tape_center_controller_state, &controller,
            tape_center_error, dt_s);
    command_vy = tape_center_correction_mps;
}

bool start_body(float vx, float vy, float omega, uint32_t duration_ms,
                TestMode requested_mode, const char *name) {
    esp_err_t error = coast_all();
    if (error == ESP_OK) {
        error = drivetrain_set_body_velocity(&drivetrain, vx, vy, omega);
    }
    if (error != ESP_OK) {
        report_error("body command", error);
        return false;
    }

    command_vx = vx;
    command_vy = vy;
    command_omega = omega;
    applied_vx = applied_vy = applied_omega = 0.0f;
    motion_end_ms = millis() + duration_ms;
    mode = requested_mode;
    sequence_step_active = requested_mode == TestMode::SEQUENCE;
    Serial.printf("# START %s: vx=%.3f vy=%.3f omega=%.3f for %lu ms\n",
                  name, vx, vy, omega, static_cast<unsigned long>(duration_ms));
    return true;
}

void start_named_translation(const String &name, float speed, uint32_t duration_ms) {
    float vx = 0.0f;
    float vy = 0.0f;
    direction_vector(name, &vx, &vy);
    vx *= speed;
    vy *= speed;
    start_body(vx, vy, 0.0f, duration_ms, TestMode::BODY, name.c_str());
}

void start_distance(const String &direction, float distance_m, float max_speed_mps) {
    float unit_x = 0.0f;
    float unit_y = 0.0f;
    if (!direction_vector(direction, &unit_x, &unit_y)) {
        Serial.println("# invalid distance direction");
        return;
    }
    const esp_err_t error = coast_all();
    if (error != ESP_OK) {
        report_error("distance start", error);
        return;
    }
    capture_motion_start_counts();
    motion_direction_x = unit_x;
    motion_direction_y = unit_y;
    motion_target = distance_m;
    motion_max_speed = max_speed_mps;
    command_vx = command_vy = command_omega = 0.0f;
    applied_vx = applied_vy = applied_omega = 0.0f;
    position_timeout_at_ms = millis() + motion_timeout_ms;
    mode = TestMode::DISTANCE;
    Serial.printf("# START distance %s: target=%.4f m max_speed=%.3f m/s timeout=%lu ms\n",
                  direction.c_str(), distance_m, max_speed_mps,
                  static_cast<unsigned long>(motion_timeout_ms));
}

void start_angle(const String &direction, float angle_deg, float max_omega_rad_s) {
    const esp_err_t error = coast_all();
    if (error != ESP_OK) {
        report_error("angle start", error);
        return;
    }
    capture_motion_start_counts();
    motion_turn_sign = direction == "cw" ? -1 : 1;
    motion_target = angle_deg * kPi / 180.0f;
    motion_max_speed = max_omega_rad_s;
    command_vx = command_vy = command_omega = 0.0f;
    applied_vx = applied_vy = applied_omega = 0.0f;
    position_timeout_at_ms = millis() + motion_timeout_ms;
    mode = TestMode::ANGLE;
    Serial.printf("# START turn-angle %s: target=%.2f deg max_omega=%.3f rad/s timeout=%lu ms\n",
                  direction.c_str(), angle_deg, max_omega_rad_s,
                  static_cast<unsigned long>(motion_timeout_ms));
}

void service_position_goal(uint32_t now_ms) {
    if (time_reached(now_ms, position_timeout_at_ms)) {
        begin_controlled_stop("position command timed out");
        return;
    }

    DrivetrainBodyVelocity relative = {};
    const esp_err_t error = get_relative_body_motion(&relative);
    if (error != ESP_OK) {
        report_error("relative motion", error);
        return;
    }

    if (mode == TestMode::DISTANCE) {
        motion_progress = relative.vx * motion_direction_x +
                          relative.vy * motion_direction_y;
        const float remaining = motion_target - motion_progress;
        if (remaining <= distance_tolerance_m) {
            begin_controlled_stop("distance reached");
            return;
        }
        const float stopping_speed = sqrtf(fmaxf(
            0.0f, 2.0f * deceleration_per_s2 *
            (remaining - distance_tolerance_m)));
        const float requested_speed = fminf(motion_max_speed, stopping_speed);
        command_vx = motion_direction_x * requested_speed;
        command_vy = motion_direction_y * requested_speed;
        command_omega = 0.0f;
    } else if (mode == TestMode::ANGLE) {
        motion_progress = static_cast<float>(motion_turn_sign) * relative.omega;
        const float remaining = motion_target - motion_progress;
        if (remaining <= angle_tolerance_rad) {
            begin_controlled_stop("angle reached");
            return;
        }
        const float stopping_omega = sqrtf(fmaxf(
            0.0f, 2.0f * deceleration_per_s2 *
            (remaining - angle_tolerance_rad)));
        command_vx = command_vy = 0.0f;
        command_omega = static_cast<float>(motion_turn_sign) *
                        fminf(motion_max_speed, stopping_omega);
    }
}

void start_pair(int index, float duty, uint32_t duration_ms) {
    if (fabsf(duty) > kMaxPairDuty || duty == 0.0f || !isfinite(duty)) {
        Serial.printf("# pair duty must be nonzero and within +/-%.2f\n", kMaxPairDuty);
        return;
    }
    esp_err_t error = coast_all();
    if (error == ESP_OK) {
        error = encoder_driver_reset(&drivetrain.devices.encoders[index]);
    }
    if (error == ESP_OK) {
        error = motor_driver_set_duty(&drivetrain.devices.motors[index], duty);
    }
    if (error != ESP_OK) {
        report_error("pair command", error);
        return;
    }
    pair_index = index;
    motion_end_ms = millis() + duration_ms;
    mode = TestMode::PAIR;
    Serial.printf("# START pair %s: duty=%.3f for %lu ms; encoder velocity should have the same sign\n",
                  kWheelNames[index], duty, static_cast<unsigned long>(duration_ms));
}

void start_sequence_step() {
    const SequenceStep &step = kSequence[sequence_index];
    const float vx = step.vx_factor * sequence_speed_mps;
    const float vy = step.vy_factor * sequence_speed_mps;
    const float omega = step.omega_factor * sequence_turn_rad_s;
    start_body(vx, vy, omega, sequence_step_ms, TestMode::SEQUENCE, step.name);
}

void start_sequence(float speed, uint32_t duration_ms) {
    sequence_speed_mps = speed;
    sequence_step_ms = duration_ms;
    sequence_index = 0;
    start_sequence_step();
}

void process_command(String line) {
    line.trim();
    line.toLowerCase();
    if (line.length() == 0) return;

    String tokens[6];
    const int count = split_tokens(line, tokens, 6);
    const String &command = tokens[0];
    if (command == "help") { print_help(); return; }
    if (command == "config") { print_config(); return; }
    if (command == "status") { print_telemetry(millis()); return; }
    if (command == "tape") {
        if (sample_tape_sensors()) print_tape_telemetry(millis());
        else Serial.println("# tape sensors are not ready");
        return;
    }
    if (command == "limits" || command == "settings") {
        print_runtime_settings();
        print_controller_config();
        print_tape_tuning();
        return;
    }
    if (command == "enable") { enable_test(); return; }
    if (command == "brake") { brake_test(); return; }
    if (command == "coast") { stop_test(); return; }
    if (command == "stop") {
        if (drivetrain_ready) begin_controlled_stop("operator command");
        else Serial.println("# drivetrain is not ready");
        return;
    }
    if (command == "reset-encoders") { reset_all_encoders(); return; }

    if (command == "pi") {
        if (count == 1 || tokens[1] == "show") { print_controller_config(); return; }
        if (tokens[1] == "reset") {
            const int wheel = count >= 3 ? parse_wheel(tokens[2]) : -1;
            if (count >= 3 && wheel < 0) { Serial.println("# invalid wheel"); return; }
            reset_controller_config(wheel);
            return;
        }
        if (count < 4) {
            Serial.println("# usage: pi <wheel|all> kff|offset|kp|ki|slew <value>");
            return;
        }
        const int wheel = tokens[1] == "all" ? -1 : parse_wheel(tokens[1]);
        if (wheel < 0 && tokens[1] != "all") { Serial.println("# invalid wheel"); return; }
        update_pi_parameter(wheel, tokens[2], tokens[3].toFloat());
        return;
    }

    if (command == "tape-tune") {
        if (count == 1 || tokens[1] == "show") { print_tape_tuning(); return; }
        if (tokens[1] == "reset") {
            reset_tape_tuning();
            return;
        }
        if (count < 3) {
            Serial.println("# usage: tape-tune <field> <value>");
            return;
        }
        update_tape_parameter(tokens[1], tokens[2].toFloat());
        return;
    }

    if (command == "ramp") {
        if (count < 3) { Serial.println("# usage: ramp <accel> <decel>"); return; }
        update_ramp_tuning(tokens[1].toFloat(), tokens[2].toFloat());
        return;
    }

    if (command == "distance-tolerance") {
        if (count < 2) { Serial.println("# usage: distance-tolerance <meters>"); return; }
        update_distance_tolerance_tuning(tokens[1].toFloat());
        return;
    }

    if (command == "angle-tolerance") {
        if (count < 2) { Serial.println("# usage: angle-tolerance <degrees>"); return; }
        update_angle_tolerance_tuning(tokens[1].toFloat());
        return;
    }

    if (command == "telemetry") {
        update_telemetry_period_tuning(count >= 2 ? tokens[1].toInt() : 0);
        return;
    }

    if (command == "timeout") {
        update_motion_timeout_tuning(count >= 2 ? tokens[1].toInt() : 0);
        return;
    }

    // Inversion only edits the harness's RAM-backed configuration. Keep it
    // available after a control fault so its result can still be inspected
    // with "config" before rebooting to restore motor operation.
    if (command == "invert") {
        if (count < 3) { Serial.println("# usage: invert motor|encoder <wheel> [0|1]"); return; }
        const int wheel = parse_wheel(tokens[2]);
        if (wheel < 0) { Serial.println("# invalid wheel"); return; }
        const String *value = count >= 4 ? &tokens[3] : nullptr;
        set_inversion(tokens[1], wheel, value);
        return;
    }

    if (!drivetrain_ready) {
        Serial.println("# drivetrain is not ready; inversion is saved in RAM, reboot before motion");
        return;
    }

    if (command == "pair") {
        if (count < 2) { Serial.println("# usage: pair <wheel> <duty> [ms]"); return; }
        const int wheel = parse_wheel(tokens[1]);
        if (wheel < 0) { Serial.println("# wheel must be fl|fr|bl|br|1|2|3|4"); return; }
        const float duty = count >= 3 ? tokens[2].toFloat() : kDefaultPairDuty;
        uint32_t duration = kDefaultDurationMs;
        if (count >= 4 && !parse_duration(tokens[3], duration)) return;
        start_pair(wheel, duty, duration);
        return;
    }

    if (command == "tape-center") {
        if (count < 2 || (tokens[1] != "front" && tokens[1] != "back")) {
            Serial.println("# usage: tape-center front|back [max_mps] [ms] [polarity]");
            return;
        }
        const float maximum_strafe = count >= 3
            ? tokens[2].toFloat() : kDefaultTapeCenterMaxMps;
        uint32_t duration = kDefaultDurationMs;
        if (count >= 4 && !parse_duration(
                tokens[3], duration, kMaxTapeDurationMs)) return;
        const float polarity = count >= 5 ? tokens[4].toFloat() : 1.0f;
        if (!isfinite(maximum_strafe) || maximum_strafe <= 0.0f ||
            maximum_strafe > test_config.max_vy_mps) {
            Serial.printf("# tape-center max speed must be >0 and <=%.2f m/s\n",
                          test_config.max_vy_mps);
            return;
        }
        if (polarity != 1.0f && polarity != -1.0f) {
            Serial.println("# tape-center polarity must be 1 or -1");
            return;
        }
        start_tape_center(tokens[1] == "front" ? 0 : 1,
                          maximum_strafe, duration, polarity);
        return;
    }

    if (command == "tape-follow") {
        if (count < 2 || (tokens[1] != "front" && tokens[1] != "back")) {
            Serial.println("# usage: tape-follow front|back [travel_mps] [max_strafe_mps] [ms] [polarity]");
            return;
        }
        const bool moving_forward = tokens[1] == "front";
        const float travel_speed = count >= 3
            ? tokens[2].toFloat() : kDefaultTapeFollowSpeedMps;
        const float maximum_strafe = count >= 4
            ? tokens[3].toFloat() : kDefaultTapeCenterMaxMps;
        uint32_t duration = kDefaultDurationMs;
        if (count >= 5 && !parse_duration(
                tokens[4], duration, kMaxTapeDurationMs)) return;
        const float polarity = count >= 6 ? tokens[5].toFloat() : 1.0f;
        if (!isfinite(travel_speed) || travel_speed <= 0.0f ||
            travel_speed > test_config.max_vx_mps) {
            Serial.printf("# tape-follow travel speed must be >0 and <=%.2f m/s\n",
                          test_config.max_vx_mps);
            return;
        }
        if (!isfinite(maximum_strafe) || maximum_strafe <= 0.0f ||
            maximum_strafe > test_config.max_vy_mps) {
            Serial.printf("# tape-follow max strafe must be >0 and <=%.2f m/s\n",
                          test_config.max_vy_mps);
            return;
        }
        if (polarity != 1.0f && polarity != -1.0f) {
            Serial.println("# tape-follow polarity must be 1 or -1");
            return;
        }
        start_tape_center(moving_forward ? 0 : 1, maximum_strafe,
                          duration, polarity,
                          moving_forward ? travel_speed : -travel_speed);
        return;
    }

    if (command == "distance") {
        if (count < 3) {
            Serial.println("# usage: distance <direction> <meters> [max_speed_mps]");
            return;
        }
        const float distance_m = tokens[2].toFloat();
        const float speed = count >= 4 ? tokens[3].toFloat() : kDefaultMoveSpeedMps;
        if (!isfinite(distance_m) || distance_m <= distance_tolerance_m ||
            distance_m > 100.0f) {
            Serial.println("# distance must exceed tolerance and be <=100 m");
            return;
        }
        if (!isfinite(speed) || speed <= 0.0f || speed > test_config.max_vx_mps) {
            Serial.printf("# speed must be >0 and <=%.2f m/s\n", test_config.max_vx_mps);
            return;
        }
        start_distance(tokens[1], distance_m, speed);
        return;
    }

    if (command == "turn-angle") {
        if (count < 3 || (tokens[1] != "cw" && tokens[1] != "ccw")) {
            Serial.println("# usage: turn-angle cw|ccw <degrees> [max_rad_s]");
            return;
        }
        const float angle = tokens[2].toFloat();
        const float omega = count >= 4 ? tokens[3].toFloat() : kDefaultTurnSpeedRadS;
        if (!isfinite(angle) || angle <= angle_tolerance_rad * 180.0f / kPi ||
            angle > 3600.0f) {
            Serial.println("# angle must exceed tolerance and be <=3600 degrees");
            return;
        }
        if (!isfinite(omega) || omega <= 0.0f || omega > test_config.max_omega_rad_s) {
            Serial.printf("# turn speed must be >0 and <=%.2f rad/s\n",
                          test_config.max_omega_rad_s);
            return;
        }
        start_angle(tokens[1], angle, omega);
        return;
    }

    const bool translation = command == "forward" || command == "backward" ||
        command == "left" || command == "right" || command == "forward-left" ||
        command == "forward-right" || command == "backward-left" ||
        command == "backward-right";
    if (translation) {
        const float speed = count >= 2 ? tokens[1].toFloat() : kDefaultMoveSpeedMps;
        uint32_t duration = kDefaultDurationMs;
        if (!isfinite(speed) || speed <= 0.0f || speed > test_config.max_vx_mps) {
            Serial.printf("# speed must be >0 and <=%.2f m/s\n", test_config.max_vx_mps);
            return;
        }
        if (count >= 3 && !parse_duration(tokens[2], duration)) return;
        start_named_translation(command, speed, duration);
        return;
    }

    if (command == "cw" || command == "ccw") {
        const float omega = count >= 2 ? tokens[1].toFloat() : kDefaultTurnSpeedRadS;
        uint32_t duration = kDefaultDurationMs;
        if (!isfinite(omega) || omega <= 0.0f || omega > test_config.max_omega_rad_s) {
            Serial.printf("# turn speed must be >0 and <=%.2f rad/s\n", test_config.max_omega_rad_s);
            return;
        }
        if (count >= 3 && !parse_duration(tokens[2], duration)) return;
        start_body(0.0f, 0.0f, command == "cw" ? -omega : omega,
                   duration, TestMode::BODY, command.c_str());
        return;
    }

    if (command == "sequence") {
        const float speed = count >= 2 ? tokens[1].toFloat() : kDefaultMoveSpeedMps;
        uint32_t duration = kDefaultDurationMs;
        if (!isfinite(speed) || speed <= 0.0f || speed > test_config.max_vx_mps) {
            Serial.printf("# sequence speed must be >0 and <=%.2f m/s\n", test_config.max_vx_mps);
            return;
        }
        if (count >= 3 && !parse_duration(tokens[2], duration)) return;
        start_sequence(speed, duration);
        return;
    }

    Serial.println("# unknown command; type help");
}

// Provides terminal-style input editing without relying on monitor-side echo.
// Both common Backspace bytes are handled, CR is ignored, and LF executes the
// completed line. Keeping the edit buffer here also prevents deletion bytes
// from becoming invisible characters inside command names or arguments.
void service_serial_input() {
    while (Serial.available()) {
        const char input = static_cast<char>(Serial.read());

        if (input == '\r') continue;
        if (input == '\n') {
            Serial.println();
            const String completed_line = serial_command_line;
            serial_command_line = "";
            process_command(completed_line);
            Serial.print("> ");
            continue;
        }

        if (input == '\b' || input == 0x7f) {
            if (serial_command_line.length() > 0) {
                serial_command_line.remove(serial_command_line.length() - 1);
                Serial.print("\b \b");
            }
            continue;
        }

        if (input >= 0x20 && input <= 0x7e) {
            if (serial_command_line.length() < kMaxCommandLength) {
                serial_command_line += input;
                Serial.write(input);
            } else {
                Serial.write('\a');
            }
        }
    }
}

void initialize_test_config() {
    test_config = DRIVETRAIN_CONFIG;
    live_tape_config = TAPE_FOLLOWER_CONFIG;
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        test_motor_configs[i] = *DRIVETRAIN_CONFIG.motor_configs[i];
        test_encoder_configs[i] = *DRIVETRAIN_CONFIG.encoder_configs[i];
        test_config.motor_configs[i] = &test_motor_configs[i];
        test_config.encoder_configs[i] = &test_encoder_configs[i];
    }
}

void initialize_tape_sensors() {
    esp_err_t error = tape_sensor_mux_init(
        &tape_sensor_mux, &TAPE_SENSOR_MUX_CONFIG);
    if (error == ESP_OK) {
        error = tape_sensor_driver_init(
            &tape_sensors[0], &FRONT_TAPE_SENSOR_CONFIG, &tape_sensor_mux);
    }
    if (error == ESP_OK) {
        error = tape_sensor_driver_init(
            &tape_sensors[1], &BACK_TAPE_SENSOR_CONFIG, &tape_sensor_mux);
    }
    if (error == ESP_OK) {
        error = tape_sensor_driver_init(
            &tape_sensors[2], &LEFT_TAPE_SENSOR_CONFIG, &tape_sensor_mux);
    }
    if (error != ESP_OK) {
        Serial.printf("# tape sensor initialization failed: %s\n",
                      esp_err_to_name(error));
        tape_sensors_ready = false;
        return;
    }
    tape_sensors_ready = true;
    sample_tape_sensors();
    Serial.println("# tape sensors ready; channel order is left-to-right");
}

void update_open_loop_encoders() {
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        const esp_err_t error = encoder_driver_update(&drivetrain.devices.encoders[i]);
        if (error != ESP_OK) {
            report_error("encoder update", error);
            return;
        }
    }
}

void service_sequence(uint32_t now_ms) {
    if (sequence_step_active && time_reached(now_ms, motion_end_ms)) {
        const esp_err_t error = coast_all();
        if (error != ESP_OK) { report_error("sequence coast", error); return; }
        sequence_step_active = false;
        next_sequence_ms = now_ms + kSequencePauseMs;
        Serial.println("# sequence pause");
    } else if (!sequence_step_active && time_reached(now_ms, next_sequence_ms)) {
        ++sequence_index;
        if (sequence_index >= kSequenceLength) {
            stop_test(false);
            Serial.println("# sequence complete; all motors coasting");
        } else {
            start_sequence_step();
        }
    }
}


}  // namespace

void setup() {
    Serial.begin(115200);
    Serial.setTimeout(20);
    delay(1000);
    Serial.println("\n# Initializing drivetrain hardware test...");

    initialize_test_config();
    initialize_tape_sensors();
    esp_err_t error = drivetrain_init(&drivetrain, &test_config);
    if (error == ESP_OK) error = drivetrain_enable(&drivetrain);
    if (error != ESP_OK) {
        Serial.printf("# drivetrain initialization failed: %s\n", esp_err_to_name(error));
        return;
    }

    drivetrain_ready = true;
    last_control_us = esp_timer_get_time();
    print_config();
    print_help();
    Serial.print("> ");
}

void loop() {
    service_serial_input();
    const uint32_t now_ms = millis();
    if (tape_sensors_ready &&
        now_ms - last_tape_sample_ms >= kTapeSamplePeriodMs) {
        last_tape_sample_ms = now_ms;
        sample_tape_sensors();
    }
    if (!drivetrain_ready) {
        if (serial_command_line.length() == 0 &&
            now_ms - last_telemetry_ms >= telemetry_period_ms) {
            last_telemetry_ms = now_ms;
            print_tape_telemetry(now_ms);
        }
        delay(1);
        return;
    }

    if (mode == TestMode::PAIR && time_reached(now_ms, motion_end_ms)) {
        stop_test();
    } else if (mode == TestMode::BODY && time_reached(now_ms, motion_end_ms)) {
        begin_controlled_stop("timed movement complete");
    } else if (mode == TestMode::SEQUENCE) {
        service_sequence(now_ms);
    } else if (mode == TestMode::DISTANCE || mode == TestMode::ANGLE) {
        service_position_goal(now_ms);
    } else if (mode == TestMode::TAPE_CENTER &&
               time_reached(now_ms, motion_end_ms)) {
        begin_controlled_stop("tape-center duration complete");
    } else if (mode == TestMode::STOPPING &&
               (wheels_are_stopped() || time_reached(now_ms, motion_end_ms))) {
        stop_test(false);
        Serial.println("# controlled stop complete; all motors coasting");
    }

    const int64_t now_us = esp_timer_get_time();
    if (now_us - last_control_us >= kControlPeriodUs) {
        const float dt_s = static_cast<float>(now_us - last_control_us) / 1000000.0f;
        last_control_us = now_us;
        esp_err_t error = ESP_OK;
        const bool closed_loop = mode_uses_closed_loop(mode);
        if (closed_loop) {
            if (mode == TestMode::TAPE_CENTER) service_tape_center(dt_s);
            update_ramped_command(dt_s);
            error = drivetrain_set_body_velocity(
                &drivetrain, applied_vx, applied_vy, applied_omega);
            // set_body_velocity records its own current timestamp. Fetch a
            // fresh time afterward so update never receives an earlier time.
            if (error == ESP_OK) {
                error = drivetrain_update(&drivetrain, esp_timer_get_time());
            }
        } else {
            error = ESP_OK;
            update_open_loop_encoders();
            if (!drivetrain_ready) return;
        }
        if (error != ESP_OK) { report_error("control update", error); return; }
    }

    if (serial_command_line.length() == 0 &&
        now_ms - last_telemetry_ms >= telemetry_period_ms) {
        last_telemetry_ms = now_ms;
        if (mode == TestMode::IDLE) print_tape_telemetry(now_ms);
        else print_telemetry(now_ms);
    }
    delay(1);
}

namespace {

// -----------------------------------------------------------------------------
// Real-time tuning only
// All changes in this section are RAM-only and exist for hardware calibration.
// Copy accepted values into the compiled configuration to make them persistent.
// -----------------------------------------------------------------------------

void print_runtime_settings() {
    Serial.printf("# limits translation=%.3f_mps turn=%.3f_rad_s duty=%.3f pair_duty=%.3f\n",
                  test_config.max_vx_mps, test_config.max_omega_rad_s,
                  test_config.max_duty, kMaxPairDuty);
    Serial.printf("# runtime ramp_accel=%.3f ramp_decel=%.3f distance_tolerance=%.4f_m "
                  "angle_tolerance=%.2f_deg timeout=%lu_ms telemetry=%lu_ms\n",
                  acceleration_per_s2, deceleration_per_s2,
                  distance_tolerance_m, angle_tolerance_rad * 180.0f / kPi,
                  static_cast<unsigned long>(motion_timeout_ms),
                  static_cast<unsigned long>(telemetry_period_ms));
}

void print_config() {
    Serial.println("# wheel,motor_inverted,encoder_inverted");
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        Serial.printf("# %s,%d,%d\n", kWheelNames[i],
                      test_motor_configs[i].direction_inverted,
                      test_encoder_configs[i].direction_inverted);
    }
}

void update_ramp_tuning(float acceleration, float deceleration) {
    if (!isfinite(acceleration) || !isfinite(deceleration) ||
        acceleration <= 0.0f || deceleration <= 0.0f ||
        acceleration > 10.0f || deceleration > 10.0f) {
        Serial.println("# ramp values must be >0 and <=10");
        return;
    }
    acceleration_per_s2 = acceleration;
    deceleration_per_s2 = deceleration;
    print_runtime_settings();
}

void update_distance_tolerance_tuning(float value_m) {
    if (!isfinite(value_m) || value_m <= 0.0f || value_m > 0.5f) {
        Serial.println("# distance tolerance must be >0 and <=0.5 m");
        return;
    }
    distance_tolerance_m = value_m;
    print_runtime_settings();
}

void update_angle_tolerance_tuning(float value_deg) {
    if (!isfinite(value_deg) || value_deg <= 0.0f || value_deg > 45.0f) {
        Serial.println("# angle tolerance must be >0 and <=45 degrees");
        return;
    }
    angle_tolerance_rad = value_deg * kPi / 180.0f;
    print_runtime_settings();
}

void update_telemetry_period_tuning(long value_ms) {
    if (value_ms < 20 || value_ms > 5000) {
        Serial.println("# telemetry period must be 20..5000 ms");
        return;
    }
    telemetry_period_ms = static_cast<uint32_t>(value_ms);
    print_runtime_settings();
}

void update_motion_timeout_tuning(long value_ms) {
    if (value_ms < 100 || value_ms > static_cast<long>(kMaxMotionTimeoutMs)) {
        Serial.printf("# timeout must be 100..%lu ms\n",
                      static_cast<unsigned long>(kMaxMotionTimeoutMs));
        return;
    }
    motion_timeout_ms = static_cast<uint32_t>(value_ms);
    print_runtime_settings();
}

void print_controller_config() {
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        drivetrain_get_motor_controller_config(
            &drivetrain, static_cast<DrivetrainMotorId>(i),
            &live_controller_configs[i]);
        const WheelVelocityControllerConfig &config = live_controller_configs[i];
        Serial.printf("# PI %s kff=%.5f offset=%.5f kp=%.5f ki=%.5f slew=%.5f\n",
                      kWheelNames[i], config.kff, config.kff_offset,
                      config.kp, config.ki, config.duty_slew_per_s);
    }
}

void print_tape_tuning() {
    Serial.printf("# TAPE kp=%.4f ki=%.4f kd=%.4f integral_limit=%.3f "
                  "heading=%.3f max_omega=%.3f angular_accel=%.3f "
                  "search_omega=%.3f lost_timeout=%.3f dt_max=%.3f\n",
                  live_tape_config.controller.proportional_gain,
                  live_tape_config.controller.integral_gain,
                  live_tape_config.controller.derivative_gain,
                  live_tape_config.controller.integral_limit,
                  live_tape_config.heading.gain_s_inv,
                  live_tape_config.heading.max_omega_rad_s,
                  live_tape_config.heading.max_acceleration_rad_s2,
                  live_tape_config.search.angular_velocity_rad_s,
                  live_tape_config.search.timeout_s,
                  live_tape_config.controller_dt_max_s);
}

void set_inversion(const String &kind, int wheel, const String *value) {
    stop_test(false);
    bool *setting = nullptr;
    if (kind == "motor") setting = &test_motor_configs[wheel].direction_inverted;
    if (kind == "encoder") setting = &test_encoder_configs[wheel].direction_inverted;
    if (setting == nullptr) {
        Serial.println("# usage: invert motor|encoder <wheel> [0|1]");
        return;
    }

    *setting = value == nullptr ? !*setting : value->toInt() != 0;
    if (kind == "encoder") {
        const esp_err_t error = encoder_driver_reset(&drivetrain.devices.encoders[wheel]);
        if (error != ESP_OK) {
            report_error("encoder reset", error);
            return;
        }
    }
    Serial.printf("# %s inversion for %s = %d (RAM only)\n",
                  kind.c_str(), kWheelNames[wheel], *setting);
}

void update_pi_parameter(int wheel, const String &field, float value) {
    if (!isfinite(value)) {
        Serial.println("# PI value must be finite");
        return;
    }
    const int first = wheel < 0 ? 0 : wheel;
    const int last = wheel < 0 ? DRIVETRAIN_MOTOR_MAX : wheel + 1;
    for (int index = first; index < last; ++index) {
        drivetrain_get_motor_controller_config(
            &drivetrain, static_cast<DrivetrainMotorId>(index),
            &live_controller_configs[index]);
        WheelVelocityControllerConfig &config = live_controller_configs[index];
        if (field == "kff") config.kff = value;
        else if (field == "offset") config.kff_offset = value;
        else if (field == "kp") config.kp = value;
        else if (field == "ki") config.ki = value;
        else if (field == "slew") config.duty_slew_per_s = value;
        else {
            Serial.println("# PI field must be kff|offset|kp|ki|slew");
            return;
        }
        const esp_err_t error = drivetrain_set_motor_controller_config(
            &drivetrain, static_cast<DrivetrainMotorId>(index), &config);
        if (error != ESP_OK) {
            Serial.printf("# PI update rejected for %s: %s\n",
                          kWheelNames[index], esp_err_to_name(error));
            return;
        }
    }
    print_controller_config();
}

void reset_controller_config(int wheel) {
    const int first = wheel < 0 ? 0 : wheel;
    const int last = wheel < 0 ? DRIVETRAIN_MOTOR_MAX : wheel + 1;
    for (int index = first; index < last; ++index) {
        live_controller_configs[index] = test_config.wheel_controller;
        const esp_err_t error = drivetrain_set_motor_controller_config(
            &drivetrain, static_cast<DrivetrainMotorId>(index),
            &live_controller_configs[index]);
        if (error != ESP_OK) {
            Serial.printf("# PI reset failed: %s\n", esp_err_to_name(error));
            return;
        }
    }
    Serial.println("# PI restored to compiled defaults");
    print_controller_config();
}

void update_tape_parameter(const String &field, float value) {
    if (!isfinite(value)) {
        Serial.println("# tape value must be finite");
        return;
    }

    TapeFollowerConfig candidate = live_tape_config;
    if (field == "kp") candidate.controller.proportional_gain = value;
    else if (field == "ki") candidate.controller.integral_gain = value;
    else if (field == "kd") candidate.controller.derivative_gain = value;
    else if (field == "integral-limit") candidate.controller.integral_limit = value;
    else if (field == "heading") candidate.heading.gain_s_inv = value;
    else if (field == "max-omega") candidate.heading.max_omega_rad_s = value;
    else if (field == "angular-accel") candidate.heading.max_acceleration_rad_s2 = value;
    else if (field == "search-omega" || field == "search-speed") {
        candidate.search.angular_velocity_rad_s = value;
    }
    else if (field == "lost-timeout") candidate.search.timeout_s = value;
    else if (field == "dt-max") candidate.controller_dt_max_s = value;
    else {
        Serial.println("# unknown tape tuning field");
        return;
    }

    TapeFollower validator = {};
    if (candidate.heading.max_omega_rad_s > test_config.max_omega_rad_s ||
        tape_follower_init(&validator, &candidate) != ESP_OK) {
        Serial.println("# tape tuning update rejected");
        return;
    }
    live_tape_config = candidate;
    print_tape_tuning();
}

void reset_tape_tuning() {
    live_tape_config = TAPE_FOLLOWER_CONFIG;
    Serial.println("# tape tuning restored to compiled defaults");
    print_tape_tuning();
}

}  // namespace
