/* Implements coordinated Tower servo and stepper command handling. */
#include "control/task/tower_action_controller.h"

#include <Arduino.h>
#include <math.h>

#include <robot_common/command_packet.h>

#include "config/pin_map.h"
#include "config/servo_config.h"
#include "drivers/servo_driver.h"

namespace {

constexpr uint32_t kRotateServoSettleMs = 100;
constexpr uint32_t kClawServoSettleMs = 100;
constexpr uint32_t kHomeSettleMs = 1000;
constexpr uint32_t kLocatorExtendMs = 750; 

constexpr float kCommandDistanceUnitMm = 100.0f;

ServoDriver tower_rotate_servo = {};
ServoDriver tower_left_servo = {};
ServoDriver tower_middle_servo = {};
ServoDriver tower_right_servo = {};
volatile bool locator_contact_pending = false;
volatile bool locator_contact_armed = false;

// One-shot interrupt: the first edge disarms the switch, so bounce is ignored.
void IRAM_ATTR locator_switch_interrupt() {
    if (!locator_contact_armed) return;
    locator_contact_armed = false;
    locator_contact_pending = true;
}

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
    return command_value * kCommandDistanceUnitMm;
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
    if (!tower_action_controller_accepts(command.opcode)) return;
    if (controller_is_busy(controller)) {
        reject_if_busy(controller, command);
        Serial.printf(
            "# Tower command rejected while busy (opcode %u)\n",
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
    // opcode selects Tower X or Z; value is signed travel in 100 mm units.
    // Negative moves up/left, while positive moves down/right.
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

        case CMD_TOWER_Z:
            controller->active_stepper = controller->tower_z_stepper;
            distance_mm = requested_distance_mm(command.value);
            stepper_move_distanceMM(controller->active_stepper, distance_mm);
            start_message = "# Tower Z moving";
            break;

        case CMD_TOWER_X:
            controller->active_stepper = controller->tower_x_stepper;
            distance_mm = requested_distance_mm(command.value);
            stepper_move_distanceMM(
                controller->active_stepper, distance_mm);
            start_message = "# Tower X moving";
            break;

        case CMD_TOWER_ROTATE_HORIZONTAL:
            controller->action_is_timed = true;
            controller->action_complete_ms =
                millis() + kRotateServoSettleMs;
            servo_set_position_by_name(&tower_rotate_servo, "horizontal");
            start_message = "# Tower rotating horizontal";
            break;

        case CMD_TOWER_ROTATE_VERTICAL:
            controller->action_is_timed = true;
            controller->action_complete_ms = millis() + kRotateServoSettleMs;
            servo_set_position_by_name(&tower_rotate_servo, "vertical");
            start_message = "# Tower rotating vertical";
            break;

        case CMD_TOWER_OPEN_ALL_CLAWS:
            controller->action_is_timed = true;
            controller->action_complete_ms = millis() + kClawServoSettleMs;
            set_all_tower_claws("open");
            start_message = "# Opening left, middle, and right Tower claws";
            break;

        case CMD_TOWER_CLOSE_ALL_CLAWS:
            controller->action_is_timed = true;
            controller->action_complete_ms = millis() + kClawServoSettleMs;
            set_all_tower_claws("close");
            start_message = "# Closing left, middle, and right Tower claws";
            break;

        case CMD_TOWER_CLOSE_LEFT_CLAW:
            controller->action_is_timed = true;
            controller->action_complete_ms = millis() + kClawServoSettleMs;
            servo_set_position_by_name(&tower_left_servo, "close");
            start_message = "# Tower closing left claw";
            break;

        case CMD_TOWER_OPEN_LEFT_CLAW:
            controller->action_is_timed = true;
            controller->action_complete_ms = millis() + kClawServoSettleMs;
            servo_set_position_by_name(&tower_left_servo, "open");
            start_message = "# Tower opening left claw";
            break;

        case CMD_TOWER_CLOSE_MIDDLE_CLAW:
            controller->action_is_timed = true;
            controller->action_complete_ms = millis() + kClawServoSettleMs;
            servo_set_position_by_name(&tower_middle_servo, "close");
            start_message = "# Tower closing middle claw";
            break;

        case CMD_TOWER_OPEN_MIDDLE_CLAW:
            controller->action_is_timed = true;
            controller->action_complete_ms = millis() + kClawServoSettleMs;
            servo_set_position_by_name(&tower_middle_servo, "open");
            start_message = "# Tower opening middle claw";
            break;
        
        case CMD_TOWER_CLOSE_RIGHT_CLAW:
            controller->action_is_timed = true;
            controller->action_complete_ms = millis() + kClawServoSettleMs;
            servo_set_position_by_name(&tower_right_servo, "close");
            start_message = "# Tower closing right claw";
            break;
        
        case CMD_TOWER_OPEN_RIGHT_CLAW:
            controller->action_is_timed = true;
            controller->action_complete_ms = millis() + kClawServoSettleMs;
            servo_set_position_by_name(&tower_right_servo, "open");
            start_message = "# Tower opening right claw";
            break;

        case CMD_TOWER_EXTEND_LOCATOR:
            controller->action_is_timed = true;
            controller->action_complete_ms = millis() + kLocatorExtendMs;
            controller->locator_extended = true;
            controller->locator_contact_reported = false;
            noInterrupts();
            locator_contact_pending = digitalRead(PIN_LOC_SWITCH) == HIGH;
            locator_contact_armed = !locator_contact_pending;
            interrupts();
            digitalWrite(PIN_LOC_EN, HIGH);
            start_message = "# Tower extending locator";
            break;

        case CMD_TOWER_RETRACT_LOCATOR:
            controller->action_is_timed = true;
            controller->action_complete_ms = millis();
            controller->locator_extended = false;
            controller->locator_contact_reported = false;
            locator_contact_armed = false;
            locator_contact_pending = false;
            digitalWrite(PIN_LOC_EN, LOW);
            start_message = "# Tower retracting locator";
            break;

        default:
            return;
    }

    controller->action_active = true;
    if (controller->active_stepper != nullptr) {
        Serial.printf(
            "%s %+.0f mm\n", start_message, distance_mm);
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

    pinMode(PIN_LOC_SWITCH, INPUT_PULLUP);
    attachInterrupt(
        digitalPinToInterrupt(PIN_LOC_SWITCH),
        locator_switch_interrupt,
        RISING);

    ESP_ERROR_CHECK(initialize_tower_motors(
        controller->tower_x_stepper,
        controller->tower_z_stepper));
}

bool tower_action_controller_accepts(CommandOpcode command) {
    return command >= CMD_TOWER_HOME &&
        command <= CMD_TOWER_RETRACT_LOCATOR;
}

bool tower_action_controller_is_busy(
    const TowerActionController *controller) {
    return controller != nullptr && controller_is_busy(controller);
}

void tower_action_controller_start(
    TowerActionController *controller,
    const CommandPacket *command) {
    if (controller == nullptr || command == nullptr ||
        !tower_action_controller_accepts(command->opcode)) {
        Serial.println("# Tower command rejected: invalid action");
        return;
    }
    start_tower_action(controller, *command);
}

bool tower_action_controller_update(
    TowerActionController *controller,
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
            Serial.println("# Tower action complete");
            return true;
        }
    }

    // Poll as a fallback in case the switch's rising-edge interrupt is missed.
    if (controller->locator_extended &&
        !controller->locator_contact_reported &&
        digitalRead(PIN_LOC_SWITCH) == HIGH) {
        noInterrupts();
        locator_contact_armed = false;
        locator_contact_pending = true;
        interrupts();
    }

    // Report contact once per extension. The ISR and polling only raise the flag.
    if (locator_contact_pending && controller->locator_extended &&
        !controller->locator_contact_reported) {
        locator_contact_pending = false;
        controller->locator_contact_reported = true;
        send_status(
            controller->drivetrain_uart,
            STATUS_LOCATOR_CONTACT,
            STATUS_DETAIL_NONE);
        Serial.println("# Tower locator contacted base");
        return true;
    }
    return false;
}
