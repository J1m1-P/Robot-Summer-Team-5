/* Implements coordinated Tower servo and stepper command handling. */
#include "control/tower_action_controller.h"

#include <Arduino.h>
#include <math.h>

#include <robot_common/command_packet.h>

#include "config/pin_map.h"
#include "config/servo_config.h"
#include "drivers/servo_driver.h"

namespace {

constexpr uint32_t kStatusRepeatPeriodMs = 20;
constexpr uint32_t kStatusRepeatDurationMs = 500;
constexpr float kTowerZTravelMm = 100.0f;
constexpr float kTowerXTravelMm = 50.0f;
constexpr uint32_t kRotateServoSettleMs = 1000;
constexpr uint32_t kClawServoSettleMs = 750;
constexpr uint32_t kHomeSettleMs = 1000;
constexpr float kCommandDistanceUnitMm = 100.0f;

enum class TowerServoAction {
    ROTATE_HORIZONTAL,
    ROTATE_VERTICAL,
    OPEN_CLAW,
    CLOSE_CLAW,
};

struct TowerStepperAction {
    StepperDriver *stepper;
    float distance_mm;
    ActionStatusDetail completion_detail;
    const char *start_message;
};

struct TowerServoCommand {
    TowerServoAction action;
    ActionStatusDetail completion_detail;
    uint32_t settle_ms;
    const char *start_message;
};

ServoDriver tower_rotate_servo = {};
ServoDriver tower_left_servo = {};
ServoDriver tower_middle_servo = {};
ServoDriver tower_right_servo = {};
bool active_action_is_timed = false;
uint32_t timed_action_complete_ms = 0;

// Signed subtraction keeps this comparison valid across millis() wraparound.
bool deadline_reached(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

esp_err_t initialize_tower_servos() {
    esp_err_t error =
        servo_init(&tower_rotate_servo, towerRotateServoConfig);
    if (error != ESP_OK) return error;
    servo_set_position(&tower_rotate_servo, SERVO_POSITION_A);

    error = servo_init(&tower_left_servo, towerLeftServoConfig);
    if (error != ESP_OK) return error;
    servo_set_position(&tower_left_servo, SERVO_POSITION_B);

    error = servo_init(&tower_middle_servo, towerMiddleServoConfig);
    if (error != ESP_OK) return error;
    servo_set_position(&tower_middle_servo, SERVO_POSITION_B);

    error = servo_init(&tower_right_servo, towerRightServoConfig);
    if (error != ESP_OK) return error;
    servo_set_position(&tower_right_servo, SERVO_POSITION_B);
    return ESP_OK;
}

void set_all_claws(ServoPosition position) {
    servo_set_position(&tower_left_servo, position);
    servo_set_position(&tower_middle_servo, position);
    servo_set_position(&tower_right_servo, position);
}

// Status transmission is intentionally best effort, matching the old behavior.
void send_status(
    UartLink *drivetrain_uart,
    StatusCode code,
    uint8_t detail) {
    const StatusPacket status = {
        .code = code,
        .detail = detail,
    };
    (void)status_packet_send(drivetrain_uart, &status);
}

float requested_distance_mm(float command_value, float default_distance_mm) {
    if (command_value == 0.0f) return default_distance_mm;
    return fabsf(command_value) * kCommandDistanceUnitMm;
}

bool describe_stepper_action(
    TowerActionController *controller,
    const CommandPacket &command,
    TowerStepperAction *action) {
    switch (command.opcode) {
        case CMD_TOWER_Z_UP:
            *action = {
                controller->tower_z_stepper,
                -requested_distance_mm(command.value, kTowerZTravelMm),
                STATUS_DETAIL_TOWER_Z_RAISED,
                "# Tower Z moving up",
            };
            return true;
        case CMD_TOWER_Z_DOWN:
            *action = {
                controller->tower_z_stepper,
                requested_distance_mm(command.value, kTowerZTravelMm),
                STATUS_DETAIL_TOWER_Z_LOWERED,
                "# Tower Z moving down",
            };
            return true;
        case CMD_TOWER_X_LEFT:
            *action = {
                controller->tower_x_stepper,
                -requested_distance_mm(command.value, kTowerXTravelMm),
                STATUS_DETAIL_TOWER_X_LEFT,
                "# Tower X moving left",
            };
            return true;
        case CMD_TOWER_X_RIGHT:
            *action = {
                controller->tower_x_stepper,
                requested_distance_mm(command.value, kTowerXTravelMm),
                STATUS_DETAIL_TOWER_X_RIGHT,
                "# Tower X moving right",
            };
            return true;
        default:
            return false;
    }
}

bool describe_servo_action(
    const CommandPacket &command,
    TowerServoCommand *servo_command) {
    switch (command.opcode) {
        case CMD_TOWER_ROTATE_HORIZONTAL:
            *servo_command = {
                TowerServoAction::ROTATE_HORIZONTAL,
                STATUS_DETAIL_TOWER_HORIZONTAL,
                kRotateServoSettleMs,
                "# Tower rotating horizontal",
            };
            return true;
        case CMD_TOWER_ROTATE_VERTICAL:
            *servo_command = {
                TowerServoAction::ROTATE_VERTICAL,
                STATUS_DETAIL_TOWER_VERTICAL,
                kRotateServoSettleMs,
                "# Tower rotating vertical",
            };
            return true;
        case CMD_TOWER_OPEN_CLAW:
            *servo_command = {
                TowerServoAction::OPEN_CLAW,
                STATUS_DETAIL_TOWER_CLAW_OPEN,
                kClawServoSettleMs,
                "# Opening left, middle, and right Tower claws",
            };
            return true;
        case CMD_TOWER_CLOSE_CLAW:
            *servo_command = {
                TowerServoAction::CLOSE_CLAW,
                STATUS_DETAIL_TOWER_CLAW_CLOSED,
                kClawServoSettleMs,
                "# Closing left, middle, and right Tower claws",
            };
            return true;
        default:
            return false;
    }
}

void execute_servo_action(TowerServoAction action) {
    switch (action) {
        case TowerServoAction::ROTATE_HORIZONTAL:
            servo_set_position(
                &tower_rotate_servo, SERVO_POSITION_A);
            break;
        case TowerServoAction::ROTATE_VERTICAL:
            servo_set_position(
                &tower_rotate_servo, SERVO_POSITION_B);
            break;
        case TowerServoAction::OPEN_CLAW:
            set_all_claws(SERVO_POSITION_A);
            break;
        case TowerServoAction::CLOSE_CLAW:
            set_all_claws(SERVO_POSITION_B);
            break;
    }
}

bool controller_is_busy(TowerActionController *controller) {
    return controller->action_active ||
        stepper_is_moving(controller->tower_x_stepper) ||
        stepper_is_moving(controller->tower_z_stepper);
}

void reject_if_busy(
    TowerActionController *controller,
    const CommandPacket &command) {
    send_status(
        controller->drivetrain_uart,
        STATUS_FAULT,
        static_cast<uint8_t>(command.opcode));
}

void prepare_action(
    TowerActionController *controller,
    ActionStatusDetail completion_detail) {
    controller->active_action_detail = completion_detail;
    controller->repeated_action_detail = STATUS_DETAIL_NONE;
    controller->repeat_status_until_ms = 0;
}

void start_tower_action(
    TowerActionController *controller,
    const CommandPacket &command) {
    TowerStepperAction stepper_action = {};
    if (describe_stepper_action(controller, command, &stepper_action)) {
        if (controller_is_busy(controller)) {
            reject_if_busy(controller, command);
            return;
        }

        controller->active_stepper = stepper_action.stepper;
        prepare_action(controller, stepper_action.completion_detail);
        active_action_is_timed = false;
        stepper_move_distanceMM(
            stepper_action.stepper, stepper_action.distance_mm);
        controller->action_active = true;
        Serial.printf(
            "%s %.0f mm\n",
            stepper_action.start_message,
            fabsf(stepper_action.distance_mm));
        return;
    }

    TowerServoCommand servo_command = {};
    if (describe_servo_action(command, &servo_command)) {
        if (controller_is_busy(controller)) {
            reject_if_busy(controller, command);
            return;
        }

        controller->active_stepper = nullptr;
        prepare_action(controller, servo_command.completion_detail);
        active_action_is_timed = true;
        timed_action_complete_ms = millis() + servo_command.settle_ms;
        execute_servo_action(servo_command.action);
        controller->action_active = true;
        Serial.println(servo_command.start_message);
        return;
    }

    if (command.opcode != CMD_TOWER_HOME) return;
    if (controller_is_busy(controller)) {
        reject_if_busy(controller, command);
        return;
    }

    controller->active_stepper = nullptr;
    prepare_action(controller, STATUS_DETAIL_TOWER_HOME);
    active_action_is_timed = true;
    timed_action_complete_ms = millis() + kHomeSettleMs;
    stepper_stop(controller->tower_x_stepper);
    stepper_stop(controller->tower_z_stepper);
    digitalWrite(PIN_LOC_EN, LOW);
    servo_set_position(&tower_rotate_servo, SERVO_POSITION_A);
    set_all_claws(SERVO_POSITION_B);
    controller->action_active = true;
    Serial.println("# Tower accepting current X/Z positions as home");
}

const char *completion_message(ActionStatusDetail detail) {
    switch (detail) {
        case STATUS_DETAIL_TOWER_Z_RAISED:
            return "# Tower Z raised";
        case STATUS_DETAIL_TOWER_Z_LOWERED:
            return "# Tower Z lowered";
        case STATUS_DETAIL_TOWER_X_LEFT:
            return "# Tower X left movement complete";
        case STATUS_DETAIL_TOWER_X_RIGHT:
            return "# Tower X right movement complete";
        case STATUS_DETAIL_TOWER_HOME:
            return "# Tower home ready; locator retracted";
        case STATUS_DETAIL_TOWER_VERTICAL:
            return "# Tower vertical";
        case STATUS_DETAIL_TOWER_HORIZONTAL:
            return "# Tower horizontal";
        case STATUS_DETAIL_TOWER_CLAW_OPEN:
            return "# All Tower claws open";
        case STATUS_DETAIL_TOWER_CLAW_CLOSED:
            return "# All Tower claws closed";
        case STATUS_DETAIL_NONE:
        default:
            return "# Tower action complete";
    }
}

}  // namespace

void tower_action_controller_init(
    TowerActionController *controller,
    UartLink *drivetrain_uart,
    StepperDriver *tower_x_stepper,
    StepperDriver *tower_z_stepper) {
    controller->drivetrain_uart = drivetrain_uart;
    controller->tower_x_stepper = tower_x_stepper;
    controller->tower_z_stepper = tower_z_stepper;
    controller->active_stepper = nullptr;
    controller->action_active = false;
    controller->active_action_detail = STATUS_DETAIL_NONE;
    controller->repeated_action_detail = STATUS_DETAIL_NONE;
    controller->repeat_status_until_ms = 0;
    controller->last_status_ms = 0;
    active_action_is_timed = false;
    timed_action_complete_ms = 0;

    ESP_ERROR_CHECK(initialize_tower_servos());
}

void tower_action_controller_service_commands(
    TowerActionController *controller) {
    if (uart_link_update(controller->drivetrain_uart) != ESP_OK) return;

    PacketFrame frame = {};
    if (uart_link_take_packet(controller->drivetrain_uart, &frame) != ESP_OK ||
        !command_packet_is(&frame)) {
        return;
    }

    CommandPacket command = {};
    if (command_packet_decode(&frame, &command) != ESP_OK) return;
    start_tower_action(controller, command);
}

bool tower_action_controller_service_status(
    TowerActionController *controller,
    uint32_t now_ms) {
    bool action_complete = false;
    if (controller->action_active) {
        action_complete = active_action_is_timed
            ? deadline_reached(now_ms, timed_action_complete_ms)
            : controller->active_stepper != nullptr &&
                !stepper_is_moving(controller->active_stepper);
    }

    if (action_complete) {
        controller->action_active = false;
        active_action_is_timed = false;
        controller->repeated_action_detail =
            controller->active_action_detail;
        controller->active_action_detail = STATUS_DETAIL_NONE;
        controller->active_stepper = nullptr;
        controller->repeat_status_until_ms =
            now_ms + kStatusRepeatDurationMs;
        controller->last_status_ms = now_ms - kStatusRepeatPeriodMs;
        Serial.println(
            completion_message(controller->repeated_action_detail));
    }

    if (controller->repeated_action_detail == STATUS_DETAIL_NONE ||
        deadline_reached(now_ms, controller->repeat_status_until_ms)) {
        controller->repeated_action_detail = STATUS_DETAIL_NONE;
        return false;
    }

    if (now_ms - controller->last_status_ms >= kStatusRepeatPeriodMs) {
        controller->last_status_ms = now_ms;
        send_status(
            controller->drivetrain_uart,
            STATUS_ACTION_COMPLETE,
            static_cast<uint8_t>(controller->repeated_action_detail));
    }
    return true;
}
