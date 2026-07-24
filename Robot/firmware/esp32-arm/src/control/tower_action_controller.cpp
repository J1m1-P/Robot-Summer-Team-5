/* Implements Tower stepper command handling for the arm firmware. */
#include "control/tower_action_controller.h"

#include <Arduino.h>

#include <robot_common/command_packet.h>

namespace {

constexpr uint32_t kStatusRepeatPeriodMs = 20;
constexpr uint32_t kStatusRepeatDurationMs = 500;
constexpr float kTowerZTravelMm = 100.0f;
constexpr float kTowerXTravelMm = 50.0f;

struct TowerAction {
    StepperDriver *stepper;
    float distance_mm;
    ActionStatusDetail completion_detail;
    const char *start_message;
};

// Signed subtraction keeps this comparison valid across millis() wraparound.
bool deadline_reached(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
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

bool describe_action(
    TowerActionController *controller,
    CommandOpcode command,
    TowerAction *action) {
    switch (command) {
        case CMD_TOWER_Z_UP:
            *action = {
                controller->tower_z_stepper,
                -kTowerZTravelMm,
                STATUS_DETAIL_TOWER_Z_RAISED,
                "# Tower Z moving up 100 mm",
            };
            return true;
        case CMD_TOWER_Z_DOWN:
            *action = {
                controller->tower_z_stepper,
                kTowerZTravelMm,
                STATUS_DETAIL_TOWER_Z_LOWERED,
                "# Tower Z moving down 100 mm",
            };
            return true;
        case CMD_TOWER_X_LEFT:
            *action = {
                controller->tower_x_stepper,
                -kTowerXTravelMm,
                STATUS_DETAIL_TOWER_X_LEFT,
                "# Tower X moving left 50 mm",
            };
            return true;
        case CMD_TOWER_X_RIGHT:
            *action = {
                controller->tower_x_stepper,
                kTowerXTravelMm,
                STATUS_DETAIL_TOWER_X_RIGHT,
                "# Tower X moving right 50 mm",
            };
            return true;
        default:
            return false;
    }
}

void start_tower_action(
    TowerActionController *controller,
    CommandOpcode command) {
    TowerAction action = {};
    if (!describe_action(controller, command, &action)) return;

    if (controller->action_active || stepper_is_moving(action.stepper)) {
        send_status(
            controller->drivetrain_uart,
            STATUS_FAULT,
            static_cast<uint8_t>(command));
        return;
    }

    controller->active_stepper = action.stepper;
    controller->active_action_detail = action.completion_detail;
    controller->repeated_action_detail = STATUS_DETAIL_NONE;
    controller->repeat_status_until_ms = 0;
    stepper_move_distanceMM(action.stepper, action.distance_mm);
    controller->action_active = true;
    Serial.println(action.start_message);
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
    start_tower_action(controller, command.opcode);
}

bool tower_action_controller_service_status(
    TowerActionController *controller,
    uint32_t now_ms) {
    if (controller->action_active &&
        !stepper_is_moving(controller->active_stepper)) {
        controller->action_active = false;
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
