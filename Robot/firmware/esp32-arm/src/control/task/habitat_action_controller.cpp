/* Implements coordinated Habitat servo and stepper command handling. */
#include "control/task/habitat_action_controller.h"

#include <Arduino.h>
#include <math.h>

#include "config/servo_config.h"
#include "drivers/servo_driver.h"

namespace {

constexpr uint32_t kClawServoSettleMs = 100;
constexpr uint32_t kHomeSettleMs = 100;

constexpr float kCommandDistanceUnitMm = 100.0f;

ServoDriver habitat_left_servo = {};
ServoDriver habitat_right_servo = {};

// Signed subtraction keeps this comparison valid across millis() wraparound.
bool deadline_reached(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

esp_err_t initialize_habitat_servos() {
    esp_err_t error =
        servo_init(&habitat_left_servo, habitatLeftServoConfig);
    if (error != ESP_OK) return error;
    servo_set_position(&habitat_left_servo, SERVO_POSITION_B);

    error = servo_init(&habitat_right_servo, habitatRightServoConfig);
    if (error != ESP_OK) return error;
    servo_set_position(&habitat_right_servo, SERVO_POSITION_B);
    return ESP_OK;
}

void set_both_claws(ServoPosition position) {
    servo_set_position(&habitat_left_servo, position);
    servo_set_position(&habitat_right_servo, position);
}

// Status transmission is intentionally best effort, matching Tower behavior.
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

float requested_distance_mm(float command_value) {
    return command_value * kCommandDistanceUnitMm;
}

bool habitat_is_busy(const HabitatActionController *controller) {
    return controller->action_active ||
        stepper_is_moving(controller->habitat_x_stepper) ||
        stepper_is_moving(controller->habitat_z_stepper);
}

void reject_if_busy(
    HabitatActionController *controller,
    const CommandPacket &command) {
    send_status(
        controller->drivetrain_uart,
        STATUS_FAULT,
        static_cast<uint8_t>(command.opcode));
}

void start_habitat_action(
    HabitatActionController *controller,
    const CommandPacket &command) {

    // Failure Check
    if (!habitat_action_controller_accepts(command.opcode)) return;
    if (habitat_is_busy(controller)) {
        reject_if_busy(controller, command);
        Serial.printf(
            "# Habitat command rejected while busy (opcode %u)\n",
            static_cast<unsigned>(command.opcode));
        return;
    }

    // Clear parameters from last step
    controller->active_stepper = nullptr;
    controller->action_is_timed = false;
    controller->active_command_detail =
        static_cast<uint8_t>(
            arm_action_status_detail(command.opcode));
    float distance_mm = 0.0f;
    const char *start_message = nullptr;

    // Axis commands use both packet parameters:
    // opcode selects Habitat X or Z; value is signed travel in 100 mm units.
    // Negative moves up/left, while positive moves down/right.
    switch (command.opcode) {
        case CMD_HABITAT_HOME:
            controller->action_is_timed = true;
            controller->action_complete_ms = millis() + kHomeSettleMs;
            stepper_stop(controller->habitat_x_stepper);
            stepper_stop(controller->habitat_z_stepper);
            set_both_claws(SERVO_POSITION_B);
            start_message =
                "# Habitat accepting current X/Z positions as home";
            break;

        case CMD_HABITAT_Z:
            controller->active_stepper = controller->habitat_z_stepper;
            distance_mm = requested_distance_mm(command.value);
            stepper_move_distanceMM(
                controller->active_stepper, distance_mm);
            start_message = "# Habitat Z moving";
            break;

        case CMD_HABITAT_X:
            controller->active_stepper = controller->habitat_x_stepper;
            distance_mm = requested_distance_mm(command.value);
            stepper_move_distanceMM(
                controller->active_stepper, distance_mm);
            start_message = "# Habitat X moving";
            break;

        case CMD_HABITAT_OPEN_CLAWS:
            controller->action_is_timed = true;
            controller->action_complete_ms =
                millis() + kClawServoSettleMs;
            set_both_claws(SERVO_POSITION_A);
            start_message = "# Opening both Habitat claws";
            break;

        case CMD_HABITAT_CLOSE_CLAWS:
            controller->action_is_timed = true;
            controller->action_complete_ms =
                millis() + kClawServoSettleMs;
            set_both_claws(SERVO_POSITION_B);
            start_message = "# Closing both Habitat claws";
            break;

        case CMD_HABITAT_OPEN_LEFT_CLAW:
        case CMD_HABITAT_CLOSE_LEFT_CLAW:
            controller->action_is_timed = true;
            controller->action_complete_ms =
                millis() + kClawServoSettleMs;
            servo_set_position(
                &habitat_left_servo,
                command.opcode == CMD_HABITAT_OPEN_LEFT_CLAW
                    ? SERVO_POSITION_A
                    : SERVO_POSITION_B);
            start_message =
                command.opcode == CMD_HABITAT_OPEN_LEFT_CLAW
                    ? "# Opening left Habitat claw"
                    : "# Closing left Habitat claw";
            break;

        case CMD_HABITAT_OPEN_RIGHT_CLAW:
        case CMD_HABITAT_CLOSE_RIGHT_CLAW:
            controller->action_is_timed = true;
            controller->action_complete_ms =
                millis() + kClawServoSettleMs;
            servo_set_position(
                &habitat_right_servo,
                command.opcode == CMD_HABITAT_OPEN_RIGHT_CLAW
                    ? SERVO_POSITION_A
                    : SERVO_POSITION_B);
            start_message =
                command.opcode == CMD_HABITAT_OPEN_RIGHT_CLAW
                    ? "# Opening right Habitat claw"
                    : "# Closing right Habitat claw";
            break;

        case CMD_HABITAT_SEMI_CLOSE_LEFT_CLAW:
            controller->action_is_timed = true;
            controller->action_complete_ms =
                millis() + kClawServoSettleMs;
            servo_set_angle(
                &habitat_left_servo,
                HABITAT_LEFT_CLAW_SEMI_CLOSED_ANGLE);
            start_message = "# Semi-closing left Habitat claw";
            break;

        case CMD_HABITAT_SEMI_CLOSE_RIGHT_CLAW:
            controller->action_is_timed = true;
            controller->action_complete_ms =
                millis() + kClawServoSettleMs;
            servo_set_angle(
                &habitat_right_servo,
                HABITAT_RIGHT_CLAW_SEMI_CLOSED_ANGLE);
            start_message = "# Semi-closing right Habitat claw";
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

void habitat_action_controller_init(
    HabitatActionController *controller,
    UartLink *drivetrain_uart,
    StepperDriver *habitat_x_stepper,
    StepperDriver *habitat_z_stepper) {

    *controller = {};
    controller->drivetrain_uart = drivetrain_uart;
    controller->habitat_x_stepper = habitat_x_stepper;
    controller->habitat_z_stepper = habitat_z_stepper;

    ESP_ERROR_CHECK(initialize_habitat_servos());
}

bool habitat_action_controller_accepts(CommandOpcode command) {
    return (command >= CMD_HABITAT_HOME &&
            command <= CMD_HABITAT_CLOSE_RIGHT_CLAW) ||
        command == CMD_HABITAT_SEMI_CLOSE_LEFT_CLAW ||
        command == CMD_HABITAT_SEMI_CLOSE_RIGHT_CLAW;
}

bool habitat_action_controller_is_busy(
    const HabitatActionController *controller) {
    return controller != nullptr && habitat_is_busy(controller);
}

void habitat_action_controller_start(
    HabitatActionController *controller,
    const CommandPacket *command) {
    if (controller == nullptr || command == nullptr ||
        !habitat_action_controller_accepts(command->opcode)) {
        Serial.println("# Habitat command rejected: invalid action");
        return;
    }
    start_habitat_action(controller, *command);
}

bool habitat_action_controller_update(
    HabitatActionController *controller,
    uint32_t now_ms) {

    if (controller->action_active) {
        const bool action_complete = controller->action_is_timed
            ? deadline_reached(now_ms, controller->action_complete_ms)
            : controller->active_stepper != nullptr &&
                !stepper_is_moving(controller->active_stepper);

        if (action_complete) {
            controller->action_active = false;
            controller->active_stepper = nullptr;
            send_status(
                controller->drivetrain_uart,
                STATUS_ACTION_COMPLETE,
                controller->active_command_detail);
            Serial.println("# Habitat action complete");
            return true;
        }
    }
    return false;
}
