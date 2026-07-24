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
constexpr float kCommandValueScale = 100.0f;

// Keep these IDs synchronized with esp32-drivetrain/src/main.cpp. They travel
// in CMD_DONE.value and are echoed in STATUS_ACTION_COMPLETE.detail.
enum class TowerSequenceAction : uint8_t {
    HOME = 20,
    ROTATE_VERTICAL_FIRST = 21,
    OPEN_CLAWS = 22,
    RAISE_50_FIRST = 23,
    ROTATE_HORIZONTAL = 24,
    LOWER_50 = 25,
    CLOSE_CLAWS = 26,
    RAISE_50_SECOND = 27,
    ROTATE_VERTICAL_SECOND = 28,
    RAISE_30 = 29,
};

enum class TowerActionKind {
    STEPPER,
    HOME,
    ROTATE_HORIZONTAL,
    ROTATE_VERTICAL,
    OPEN_CLAWS,
    CLOSE_CLAWS,
};

struct TowerAction {
    TowerActionKind kind;
    StepperDriver *stepper;
    float distance_mm;
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

ActionStatusDetail sequence_completion_detail(TowerSequenceAction action) {
    return static_cast<ActionStatusDetail>(static_cast<uint8_t>(action));
}

bool describe_sequence_action(
    TowerActionController *controller,
    float encoded_action,
    TowerAction *action) {
    const long action_id = lroundf(encoded_action * kCommandValueScale);
    const TowerSequenceAction sequence_action =
        static_cast<TowerSequenceAction>(action_id);

    switch (sequence_action) {
        case TowerSequenceAction::HOME:
            *action = {
                TowerActionKind::HOME,
                nullptr,
                0.0f,
                sequence_completion_detail(sequence_action),
                kHomeSettleMs,
                "# Tower accepting current X/Z positions as home",
            };
            return true;
        case TowerSequenceAction::ROTATE_VERTICAL_FIRST:
        case TowerSequenceAction::ROTATE_VERTICAL_SECOND:
            *action = {
                TowerActionKind::ROTATE_VERTICAL,
                nullptr,
                0.0f,
                sequence_completion_detail(sequence_action),
                kRotateServoSettleMs,
                "# Tower rotating vertical",
            };
            return true;
        case TowerSequenceAction::OPEN_CLAWS:
            *action = {
                TowerActionKind::OPEN_CLAWS,
                nullptr,
                0.0f,
                sequence_completion_detail(sequence_action),
                kClawServoSettleMs,
                "# Opening left, middle, and right Tower claws",
            };
            return true;
        case TowerSequenceAction::RAISE_50_FIRST:
        case TowerSequenceAction::RAISE_50_SECOND:
            *action = {
                TowerActionKind::STEPPER,
                controller->tower_z_stepper,
                -50.0f,
                sequence_completion_detail(sequence_action),
                0,
                "# Tower Z moving up 50 mm",
            };
            return true;
        case TowerSequenceAction::ROTATE_HORIZONTAL:
            *action = {
                TowerActionKind::ROTATE_HORIZONTAL,
                nullptr,
                0.0f,
                sequence_completion_detail(sequence_action),
                kRotateServoSettleMs,
                "# Tower rotating horizontal",
            };
            return true;
        case TowerSequenceAction::LOWER_50:
            *action = {
                TowerActionKind::STEPPER,
                controller->tower_z_stepper,
                50.0f,
                sequence_completion_detail(sequence_action),
                0,
                "# Tower Z moving down 50 mm",
            };
            return true;
        case TowerSequenceAction::CLOSE_CLAWS:
            *action = {
                TowerActionKind::CLOSE_CLAWS,
                nullptr,
                0.0f,
                sequence_completion_detail(sequence_action),
                kClawServoSettleMs,
                "# Closing left, middle, and right Tower claws",
            };
            return true;
        case TowerSequenceAction::RAISE_30:
            *action = {
                TowerActionKind::STEPPER,
                controller->tower_z_stepper,
                -30.0f,
                sequence_completion_detail(sequence_action),
                0,
                "# Tower Z moving up 30 mm",
            };
            return true;
        default:
            return false;
    }
}

bool describe_legacy_action(
    TowerActionController *controller,
    CommandOpcode command,
    TowerAction *action) {
    switch (command) {
        case CMD_TOWER_Z_UP:
            *action = {
                TowerActionKind::STEPPER,
                controller->tower_z_stepper,
                -kTowerZTravelMm,
                STATUS_DETAIL_TOWER_Z_RAISED,
                0,
                "# Tower Z moving up 100 mm",
            };
            return true;
        case CMD_TOWER_Z_DOWN:
            *action = {
                TowerActionKind::STEPPER,
                controller->tower_z_stepper,
                kTowerZTravelMm,
                STATUS_DETAIL_TOWER_Z_LOWERED,
                0,
                "# Tower Z moving down 100 mm",
            };
            return true;
        case CMD_TOWER_X_LEFT:
            *action = {
                TowerActionKind::STEPPER,
                controller->tower_x_stepper,
                -kTowerXTravelMm,
                STATUS_DETAIL_TOWER_X_LEFT,
                0,
                "# Tower X moving left 50 mm",
            };
            return true;
        case CMD_TOWER_X_RIGHT:
            *action = {
                TowerActionKind::STEPPER,
                controller->tower_x_stepper,
                kTowerXTravelMm,
                STATUS_DETAIL_TOWER_X_RIGHT,
                0,
                "# Tower X moving right 50 mm",
            };
            return true;
        default:
            return false;
    }
}

bool describe_action(
    TowerActionController *controller,
    const CommandPacket &command,
    TowerAction *action) {
    if (command.opcode == CMD_DONE) {
        return describe_sequence_action(controller, command.value, action);
    }
    return describe_legacy_action(controller, command.opcode, action);
}

void execute_action(
    TowerActionController *controller,
    const TowerAction &action) {
    active_action_is_timed = action.kind != TowerActionKind::STEPPER;
    timed_action_complete_ms = millis() + action.settle_ms;

    switch (action.kind) {
        case TowerActionKind::STEPPER:
            stepper_move_distanceMM(action.stepper, action.distance_mm);
            break;
        case TowerActionKind::HOME:
            stepper_stop(controller->tower_x_stepper);
            stepper_stop(controller->tower_z_stepper);
            digitalWrite(PIN_LOC_EN, LOW);
            servo_set_position(
                &tower_rotate_servo, SERVO_POSITION_A);
            set_all_claws(SERVO_POSITION_B);
            break;
        case TowerActionKind::ROTATE_HORIZONTAL:
            servo_set_position(
                &tower_rotate_servo, SERVO_POSITION_A);
            break;
        case TowerActionKind::ROTATE_VERTICAL:
            servo_set_position(
                &tower_rotate_servo, SERVO_POSITION_B);
            break;
        case TowerActionKind::OPEN_CLAWS:
            set_all_claws(SERVO_POSITION_A);
            break;
        case TowerActionKind::CLOSE_CLAWS:
            set_all_claws(SERVO_POSITION_B);
            break;
    }
}

void start_tower_action(
    TowerActionController *controller,
    const CommandPacket &command) {
    TowerAction action = {};
    if (!describe_action(controller, command, &action)) return;

    if (controller->action_active ||
        stepper_is_moving(controller->tower_x_stepper) ||
        stepper_is_moving(controller->tower_z_stepper)) {
        send_status(
            controller->drivetrain_uart,
            STATUS_FAULT,
            static_cast<uint8_t>(command.opcode));
        return;
    }

    controller->active_stepper = action.stepper;
    controller->active_action_detail = action.completion_detail;
    controller->repeated_action_detail = STATUS_DETAIL_NONE;
    controller->repeat_status_until_ms = 0;
    execute_action(controller, action);
    controller->action_active = true;
    Serial.println(action.start_message);
}

const char *completion_message(ActionStatusDetail detail) {
    switch (static_cast<uint8_t>(detail)) {
        case STATUS_DETAIL_TOWER_Z_RAISED:
            return "# Tower Z raised";
        case STATUS_DETAIL_TOWER_Z_LOWERED:
            return "# Tower Z lowered";
        case STATUS_DETAIL_TOWER_X_LEFT:
            return "# Tower X left movement complete";
        case STATUS_DETAIL_TOWER_X_RIGHT:
            return "# Tower X right movement complete";
        case static_cast<uint8_t>(TowerSequenceAction::HOME):
            return "# Tower home ready; locator retracted";
        case static_cast<uint8_t>(
            TowerSequenceAction::ROTATE_VERTICAL_FIRST):
        case static_cast<uint8_t>(
            TowerSequenceAction::ROTATE_VERTICAL_SECOND):
            return "# Tower vertical";
        case static_cast<uint8_t>(TowerSequenceAction::OPEN_CLAWS):
            return "# All Tower claws open";
        case static_cast<uint8_t>(
            TowerSequenceAction::RAISE_50_FIRST):
        case static_cast<uint8_t>(
            TowerSequenceAction::RAISE_50_SECOND):
            return "# Tower Z raised 50 mm";
        case static_cast<uint8_t>(
            TowerSequenceAction::ROTATE_HORIZONTAL):
            return "# Tower horizontal";
        case static_cast<uint8_t>(TowerSequenceAction::LOWER_50):
            return "# Tower Z returned to home";
        case static_cast<uint8_t>(TowerSequenceAction::CLOSE_CLAWS):
            return "# All Tower claws closed";
        case static_cast<uint8_t>(TowerSequenceAction::RAISE_30):
            return "# Tower Z raised another 30 mm";
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
