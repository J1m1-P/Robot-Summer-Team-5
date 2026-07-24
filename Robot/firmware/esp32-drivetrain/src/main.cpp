/* Runs the minimal drivetrain + Tower-arm demonstration. */
#include <Arduino.h>

#include <esp_timer.h>
#include <math.h>

#include <robot_common/command_packet.h>
#include <robot_common/status_packet.h>
#include <robot_common/uart_link.h>

#include "config/communication/uart_link_config.h"
#include "config/drivetrain/drivetrain_config.h"
#include "config/drivetrain/move_config.h"
#include "control/drivetrain/drivetrain.h"
#include "control/drivetrain/move_l.h"

namespace {

constexpr uint32_t kControlPeriodUs = 5000;
constexpr uint32_t kMoveLegTimeoutMs = 15000;
constexpr uint32_t kArmActionTimeoutMs = 15000;
constexpr float kMoveSpeedMps = 0.30f;
constexpr float kMoveLegDistanceM = 0.5f;
constexpr float kPi = 3.14159265358979323846f;

enum class DemoState {
    MOVE_FIRST_METER,
    WAIT_FOR_TOWER_UP,
    WAIT_FOR_TOWER_DOWN,
    MOVE_SECOND_METER,
    WAIT_FOR_TOWER_X,
    COMPLETE,
    FAULT,
};

Drivetrain drivetrain = {};
MoveL move_l = {};
UartLink arm_uart = {};

DemoState demo_state = DemoState::FAULT;
int32_t leg_start_counts[DRIVETRAIN_MOTOR_MAX] = {};
uint32_t leg_started_ms = 0;
uint32_t arm_action_deadline_ms = 0;
int64_t last_control_us = 0;

bool deadline_reached(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

void enter_fault(const char *reason, esp_err_t error = ESP_FAIL) {
    demo_state = DemoState::FAULT;
    (void)drivetrain_brake(&drivetrain);
    Serial.printf("# DEMO FAULT: %s (%s)\n", reason, esp_err_to_name(error));
}

void capture_leg_start() {
    for (int wheel = 0; wheel < DRIVETRAIN_MOTOR_MAX; ++wheel) {
        leg_start_counts[wheel] = drivetrain_get_encoder_accumulated_count(
            &drivetrain, static_cast<DrivetrainMotorId>(wheel));
    }
}

esp_err_t get_relative_leg_motion(DrivetrainBodyVelocity *motion_out) {
    if (motion_out == nullptr) return ESP_ERR_INVALID_ARG;

    float wheel_angle[DRIVETRAIN_MOTOR_MAX] = {};
    for (int wheel = 0; wheel < DRIVETRAIN_MOTOR_MAX; ++wheel) {
        const int32_t count = drivetrain_get_encoder_accumulated_count(
            &drivetrain, static_cast<DrivetrainMotorId>(wheel));
        const EncoderDriverConfig *config = DRIVETRAIN_CONFIG.encoder_configs[wheel];
        wheel_angle[wheel] =
            static_cast<float>(count - leg_start_counts[wheel]) *
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
        &DRIVETRAIN_CONFIG.x_drive_kinematics, &wheels, motion_out);
}

esp_err_t start_move_leg(DemoState state) {
    const esp_err_t error = move_l_start(
        &move_l, &MOVE_L_CONFIG, kMoveLegDistanceM, 0.0f, kMoveSpeedMps);
    if (error != ESP_OK) return error;

    capture_leg_start();
    leg_started_ms = millis();
    last_control_us = esp_timer_get_time();
    demo_state = state;
    Serial.println(state == DemoState::MOVE_FIRST_METER
                       ? "# MoveL: first 0.5 meter"
                       : "# MoveL: second 0.5 meter");
    return ESP_OK;
}

esp_err_t send_tower_command(CommandOpcode opcode) {
    const CommandPacket command = {
        .opcode = opcode,
        .value = 0.0f,
    };
    const esp_err_t error = command_packet_send(&arm_uart, &command);
    if (error == ESP_OK) {
        arm_action_deadline_ms = millis() + kArmActionTimeoutMs;
    }
    return error;
}

void finish_first_leg() {
    esp_err_t error = drivetrain_coast(&drivetrain);
    if (error != ESP_OK) {
        enter_fault("failed to coast after first meter", error);
        return;
    }
    Serial.println("# First meter complete; drivetrain coasting");

    error = send_tower_command(CMD_TOWER_Z_UP);
    if (error != ESP_OK) {
        enter_fault("failed to request Tower up", error);
        return;
    }
    demo_state = DemoState::WAIT_FOR_TOWER_UP;
    Serial.println("# Requested Tower up");
}

void finish_second_leg() {
    esp_err_t error = drivetrain_coast(&drivetrain);
    if (error != ESP_OK) {
        enter_fault("failed to coast after second meter", error);
        return;
    }
    Serial.println("# Second meter complete; drivetrain coasting");

    error = send_tower_command(CMD_TOWER_X_RIGHT);
    if (error != ESP_OK) {
        enter_fault("failed to request Tower X movement", error);
        return;
    }
    demo_state = DemoState::WAIT_FOR_TOWER_X;
    Serial.println("# Requested Tower X movement");
}

void service_arm_uart() {
    const esp_err_t update_error = uart_link_update(&arm_uart);
    if (update_error != ESP_OK) {
        enter_fault("arm UART update failed", update_error);
        return;
    }

    PacketFrame frame = {};
    if (uart_link_take_packet(&arm_uart, &frame) != ESP_OK ||
        !status_packet_is(&frame)) {
        return;
    }

    StatusPacket status = {};
    if (status_packet_decode(&frame, &status) != ESP_OK) return;
    if (status.code == STATUS_FAULT) {
        enter_fault("arm reported a fault");
        return;
    }

    if (demo_state == DemoState::WAIT_FOR_TOWER_UP &&
        status.code == STATUS_ACTION_COMPLETE &&
        status.detail == STATUS_DETAIL_TOWER_Z_RAISED) {
        Serial.println("# Tower raised; sending separate down command");
        const esp_err_t error = send_tower_command(CMD_TOWER_Z_DOWN);
        if (error != ESP_OK) {
            enter_fault("failed to request Tower down", error);
            return;
        }
        demo_state = DemoState::WAIT_FOR_TOWER_DOWN;
    } else if (demo_state == DemoState::WAIT_FOR_TOWER_DOWN &&
               status.code == STATUS_ACTION_COMPLETE &&
               status.detail == STATUS_DETAIL_TOWER_Z_LOWERED) {
        Serial.println("# Tower lowered");
        const esp_err_t error = start_move_leg(DemoState::MOVE_SECOND_METER);
        if (error != ESP_OK) {
            enter_fault("failed to start second MoveL leg", error);
        }
    } else if (demo_state == DemoState::WAIT_FOR_TOWER_X &&
               status.code == STATUS_ACTION_COMPLETE &&
               status.detail == STATUS_DETAIL_TOWER_X_RIGHT) {
        demo_state = DemoState::COMPLETE;
        Serial.println("# Tower X moved right; DEMO COMPLETE");
    }
}

void service_move_leg(float dt_s) {
    DrivetrainBodyVelocity relative_motion = {};
    esp_err_t error = get_relative_leg_motion(&relative_motion);
    if (error != ESP_OK) {
        enter_fault("encoder odometry failed", error);
        return;
    }

    const MoveLInput input = {
        .along_track_progress_m = relative_motion.vx,
        // MoveL's correction sign is opposite the measured body +y offset.
        .cross_track_error_m = -relative_motion.vy,
        .valid = true,
    };
    MoveLOutput output = {};
    error = move_l_update(&move_l, &input, dt_s, &output);
    if (error != ESP_OK || !output.motion_valid) {
        enter_fault("MoveL control failed",
                    error == ESP_OK ? ESP_ERR_INVALID_STATE : error);
        return;
    }

    error = drivetrain_set_advanced_body_velocity(
        &drivetrain,
        output.requested_velocity.vx,
        output.requested_velocity.vy,
        output.requested_velocity.omega);
    if (error == ESP_OK) {
        error = drivetrain_update(&drivetrain, esp_timer_get_time());
    }
    if (error != ESP_OK) {
        enter_fault("drivetrain control failed", error);
        return;
    }

    if (output.status == MOVE_L_COMPLETE) {
        if (demo_state == DemoState::MOVE_FIRST_METER) finish_first_leg();
        else finish_second_leg();
    }
}

}  // namespace

void setup() {
    // Make the drivetrain inert before the serial connection grace period.
    (void)drivetrain_hold_safe_outputs(&DRIVETRAIN_CONFIG);
    Serial.begin(115200);
    delay(5000);
    Serial.println("# Starting simple drivetrain + Tower demo");

    esp_err_t error = drivetrain_init(&drivetrain, &DRIVETRAIN_CONFIG);
    if (error == ESP_OK) error = drivetrain_enable(&drivetrain);
    if (error == ESP_OK) {
        error = uart_link_init(&arm_uart, &TOP_ESP_UART_LINK_CONFIG);
    }
    if (error != ESP_OK) {
        Serial.printf("# DEMO INITIALIZATION FAILED: %s\n",
                      esp_err_to_name(error));
        (void)drivetrain_brake(&drivetrain);
        return;
    }

    error = start_move_leg(DemoState::MOVE_FIRST_METER);
    if (error != ESP_OK) enter_fault("failed to start first MoveL leg", error);
}

void loop() {
    if (demo_state == DemoState::FAULT ||
        demo_state == DemoState::COMPLETE) {
        delay(10);
        return;
    }

    service_arm_uart();
    if (demo_state == DemoState::FAULT) return;

    const uint32_t now_ms = millis();
    if ((demo_state == DemoState::WAIT_FOR_TOWER_UP ||
         demo_state == DemoState::WAIT_FOR_TOWER_DOWN ||
         demo_state == DemoState::WAIT_FOR_TOWER_X) &&
        deadline_reached(now_ms, arm_action_deadline_ms)) {
        enter_fault("arm action timed out", ESP_ERR_TIMEOUT);
        return;
    }

    const int64_t now_us = esp_timer_get_time();
    if ((demo_state == DemoState::MOVE_FIRST_METER ||
         demo_state == DemoState::MOVE_SECOND_METER) &&
        now_us - last_control_us >= kControlPeriodUs) {
        const float dt_s =
            static_cast<float>(now_us - last_control_us) / 1000000.0f;
        last_control_us = now_us;
        if (deadline_reached(now_ms, leg_started_ms + kMoveLegTimeoutMs)) {
            enter_fault("MoveL leg timed out", ESP_ERR_TIMEOUT);
            return;
        }
        service_move_leg(dt_s);
    }

    delay(1);
}
