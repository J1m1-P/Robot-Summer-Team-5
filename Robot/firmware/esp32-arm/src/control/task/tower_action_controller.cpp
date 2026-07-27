/* Implements coordinated Tower servo and stepper command handling. */
#include "control/task/tower_action_controller.h"

#include <Arduino.h>
#include <math.h>

#include <robot_common/command_packet.h>

#include "config/pin_map.h"
#include "config/servo_config.h"
#include "drivers/servo_driver.h"

namespace {

constexpr uint32_t kStatusRepeatPeriodMs = 20;
constexpr uint32_t kStatusRepeatDurationMs = 500;
constexpr uint32_t kRotateServoSettleMs = 1000;
constexpr uint32_t kClawServoSettleMs = 750;
constexpr uint32_t kHomeSettleMs = 1000;

constexpr float kCommandDistanceUnitMm = 100.0f;

ServoDriver tower_rotate_servo = {};
ServoDriver tower_left_servo = {};
ServoDriver tower_middle_servo = {};
ServoDriver tower_right_servo = {};

// Signed subtraction keeps this comparison valid across millis() wraparound.
static bool deadline_reached(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

static esp_err_t initialize_tower_motors(
    StepperDriver *tower_x_stepper,
    StepperDriver *tower_z_stepper) {
    esp_err_t error =
        servo_init(&tower_rotate_servo, towerRotateServoConfig);
    if (error != ESP_OK) return error;
    servo_set_position_by_name(&tower_rotate_servo, "horizontal");

    error = servo_init(&tower_left_servo, towerLeftServoConfig);
    if (error != ESP_OK) return error;
    servo_set_position_by_name(&tower_left_servo, "close");

    error = servo_init(&tower_middle_servo, towerMiddleServoConfig);
    if (error != ESP_OK) return error;
    servo_set_position_by_name(&tower_middle_servo, "close");

    error = servo_init(&tower_right_servo, towerRightServoConfig);
    if (error != ESP_OK) return error;
    servo_set_position_by_name(&tower_right_servo, "close");

    error = stepper_init(tower_x_stepper, towerXConfig);
    if (error != ESP_OK) return error;

    error = stepper_init(tower_z_stepper, towerZConfig);
    if (error != ESP_OK) return error;

    return ESP_OK;
}

static void set_all_tower_claws(const char *name) {
    servo_set_position_by_name(&tower_left_servo, name);
    servo_set_position_by_name(&tower_middle_servo, name);
    servo_set_position_by_name(&tower_right_servo, name);
}

// Status transmission is intentionally best effort, matching the old behavior.
static void send_status(
    UartLink *drivetrain_uart,
    StatusCode code,
    uint8_t detail) {
    const StatusPacket status = {
        .code = code,
        .detail = detail,
    };
    (void)status_packet_send(drivetrain_uart, &status);
}

static float requested_distance_mm(float command_value) {
    return fabsf(command_value) * kCommandDistanceUnitMm;
}

static bool controller_is_busy(const TowerActionController *controller) {
    return controller->action_active ||
        stepper_is_moving(controller->tower_x_stepper) ||
        stepper_is_moving(controller->tower_z_stepper);
}

static void reject_if_busy(
    TowerActionController *controller,
    const CommandPacket &command) {
    send_status(
        controller->drivetrain_uart,
        STATUS_FAULT,
        static_cast<uint8_t>(command.opcode));
}

static void start_tower_action(
    TowerActionController *controller,
    const CommandPacket &command) {

    // Failure Check
    if (command.opcode < CMD_TOWER_HOME || command.opcode >= CMD_MAX) return;
    controller->waiting_for_first_command = false;
    if (controller_is_busy(controller)) {
        reject_if_busy(controller, command);
        return;
    }

    // Clear parameters from last step
    controller->completion_pending = false;
    controller->active_stepper = nullptr;
    controller->action_is_timed = false;
    controller->active_command_detail =
        static_cast<uint8_t>(
            tower_action_status_detail(command.opcode));
    float distance_mm = 0.0f;
    const char *start_message = nullptr;

    // Decide what to do for this commmand
    switch (command.opcode) {
        case CMD_TOWER_HOME:
            controller->action_is_timed = true;
            controller->action_complete_ms = millis() + kHomeSettleMs;
            stepper_stop(controller->tower_x_stepper);
            stepper_stop(controller->tower_z_stepper);
            digitalWrite(PIN_LOC_EN, LOW);
            servo_set_position(&tower_rotate_servo, SERVO_POSITION_A);
            set_all_tower_claws("close");
            start_message =
                "# Tower accepting current X/Z positions as home";
            break;

        case CMD_TOWER_Z_UP:
            controller->active_stepper = controller->tower_z_stepper;
            distance_mm = -requested_distance_mm(command.value);
            stepper_move_distanceMM(
                controller->active_stepper, distance_mm);
            start_message = "# Tower Z moving up";
            break;

        case CMD_TOWER_Z_DOWN:
            controller->active_stepper = controller->tower_z_stepper;
            distance_mm = requested_distance_mm(command.value);
            stepper_move_distanceMM(
                controller->active_stepper, distance_mm);
            start_message = "# Tower Z moving down";
            break;

        case CMD_TOWER_X_LEFT:
            controller->active_stepper = controller->tower_x_stepper;
            distance_mm = -requested_distance_mm(command.value);
            stepper_move_distanceMM(
                controller->active_stepper, distance_mm);
            start_message = "# Tower X moving left";
            break;

        case CMD_TOWER_X_RIGHT:
            controller->active_stepper = controller->tower_x_stepper;
            distance_mm = requested_distance_mm(command.value);
            stepper_move_distanceMM(
                controller->active_stepper, distance_mm);
            start_message = "# Tower X moving right";
            break;

        case CMD_TOWER_ROTATE_HORIZONTAL:
            controller->action_is_timed = true;
            controller->action_complete_ms =
                millis() + kRotateServoSettleMs;
            servo_set_position(&tower_rotate_servo, SERVO_POSITION_A);
            start_message = "# Tower rotating horizontal";
            break;

        case CMD_TOWER_ROTATE_VERTICAL:
            controller->action_is_timed = true;
            controller->action_complete_ms =
                millis() + kRotateServoSettleMs;
            servo_set_position(&tower_rotate_servo, SERVO_POSITION_B);
            start_message = "# Tower rotating vertical";
            break;

        case CMD_TOWER_OPEN_CLAW:
            controller->action_is_timed = true;
            controller->action_complete_ms =
                millis() + kClawServoSettleMs;
            set_all_tower_claws("open");
            start_message = "# Opening left, middle, and right Tower claws";
            break;

        case CMD_TOWER_CLOSE_CLAW:
            controller->action_is_timed = true;
            controller->action_complete_ms =
                millis() + kClawServoSettleMs;
            set_all_tower_claws("close");
            start_message = "# Closing left, middle, and right Tower claws";
            break;

        default:
            return;
    }

    controller->action_active = true;
    if (controller->active_stepper != nullptr) {
        Serial.printf(
            "%s %.0f mm\n", start_message, fabsf(distance_mm));
    } else {
        Serial.println(start_message);
    }
}

}  // namespace

void tower_action_controller_init(
    TowerActionController *controller,
    UartLink *drivetrain_uart,
    StepperDriver *tower_x_stepper,
    StepperDriver *tower_z_stepper) {

    *controller = {};
    controller->drivetrain_uart = drivetrain_uart;
    controller->tower_x_stepper = tower_x_stepper;
    controller->tower_z_stepper = tower_z_stepper;
    controller->waiting_for_first_command = true;

    ESP_ERROR_CHECK(initialize_tower_motors(
        controller->tower_x_stepper,
        controller->tower_z_stepper));
}

void tower_action_controller_service_commands(
    TowerActionController *controller) {
    
    // Update the received messages
    if (uart_link_update(controller->drivetrain_uart) != ESP_OK) return;

    // Take latest UART message
    PacketFrame frame = {};
    if (uart_link_take_packet(controller->drivetrain_uart, &frame) != ESP_OK ||
        !command_packet_is(&frame)) {
        return;
    }

    // Decode the message
    CommandPacket command = {};
    if (command_packet_decode(&frame, &command) != ESP_OK) return;

    // Execute the tower action
    start_tower_action(controller, command);
}

bool tower_action_controller_update(
    TowerActionController *controller,
    uint32_t now_ms) {

    // Keep the UART dedicated to a ready signal until the drivetrain sends
    // its first command. This makes either ESP32 safe to boot or reset first.
    if (controller->waiting_for_first_command) {
        if (now_ms - controller->last_status_ms >= kStatusRepeatPeriodMs) {
            controller->last_status_ms = now_ms;
            send_status(
                controller->drivetrain_uart,
                STATUS_ACTION_COMPLETE,
                STATUS_DETAIL_NONE);
        }
        return true;
    }

    if (controller->action_active) {
        const bool action_complete = controller->action_is_timed
            ? deadline_reached(now_ms, controller->action_complete_ms)
            : controller->active_stepper != nullptr &&
                !stepper_is_moving(controller->active_stepper);
                
        if (action_complete) {
            controller->action_active = false;
            controller->completion_pending = true;
            controller->active_stepper = nullptr;
            controller->repeat_status_until_ms = now_ms + kStatusRepeatDurationMs;
            controller->last_status_ms = now_ms;
            send_status(
                controller->drivetrain_uart,
                STATUS_ACTION_COMPLETE,
                controller->active_command_detail);
            Serial.println("# Tower action complete");
            return true;
        }
    }

    if (!controller->completion_pending) return false;

    if (deadline_reached(now_ms, controller->repeat_status_until_ms)) {
        controller->completion_pending = false;
        return false;
    }

    if (now_ms - controller->last_status_ms >= kStatusRepeatPeriodMs) {
        controller->last_status_ms = now_ms;
        send_status(
            controller->drivetrain_uart,
            STATUS_ACTION_COMPLETE,
            controller->active_command_detail);
    }

    return true;
}
