/* Implements the staged rock sampling and pickup workflow. */
#include "control/task/metal_detector_action_controller.h"

#include <Arduino.h>

#include "config/servo_config.h"
#include "drivers/servo_driver.h"

namespace {

constexpr uint32_t kLiftSettleMs = 250;
constexpr uint32_t kClawSettleMs = 100;

ServoDriver rock_lift_servo = {};
ServoDriver rock_claw_servo = {};

// Handles millis() rollover while waiting for a servo to settle.
bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms) {
    return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

// Retains a result until the shared UART can accept it.
void queue_result(
    MetalDetectorActionController *controller,
    StatusCode code,
    uint8_t detail) {
    controller->stage = METAL_ACTION_IDLE;
    controller->result_code = code;
    controller->result_detail = detail;
    controller->report_pending = true;
}

// Starts a fresh detector window.
bool begin_sample(MetalDetectorActionController *controller) {
    const esp_err_t error =
        metal_detector_driver_begin_sample(controller->detector);
    if (error == ESP_OK) return true;

    Serial.printf(
        "# Metal detector sample start failed (%s)\n",
        esp_err_to_name(error));
    return false;
}

// Retracts without grabbing by lifting clear before closing the claw.
void start_empty_claw_retract(
    MetalDetectorActionController *controller,
    uint32_t now_ms,
    StatusCode result_code,
    uint8_t result_detail) {
    controller->result_code = result_code;
    controller->result_detail = result_detail;
    servo_set_angle(&rock_lift_servo, ROCK_LIFT_SEMI_LOWERED_ANGLE);
    controller->stage = METAL_ACTION_WAITING_FOR_CLEARANCE;
    controller->stage_complete_ms = now_ms + kLiftSettleMs;
}

}  // namespace

void metal_detector_action_controller_init(
    MetalDetectorActionController *controller,
    UartLink *drivetrain_uart,
    MetalDetectorDriver *detector) {
    *controller = {};
    controller->drivetrain_uart = drivetrain_uart;
    controller->detector = detector;

    ESP_ERROR_CHECK(servo_init(&rock_lift_servo, rockLiftServoConfig));
    ESP_ERROR_CHECK(servo_init(&rock_claw_servo, rockClawServoConfig));
    servo_set_position(&rock_lift_servo, SERVO_POSITION_B);
    servo_set_position(&rock_claw_servo, SERVO_POSITION_B);
}

bool metal_detector_action_controller_accepts(CommandOpcode command) {
    return command == CMD_METAL_SET_BASELINE || command == CMD_METAL_READ;
}

bool metal_detector_action_controller_is_busy(
    const MetalDetectorActionController *controller) {
    return controller != nullptr &&
        (controller->stage != METAL_ACTION_IDLE ||
         controller->report_pending);
}

void metal_detector_action_controller_start(
    MetalDetectorActionController *controller,
    const CommandPacket *command) {
    if (controller == nullptr || command == nullptr ||
        !metal_detector_action_controller_accepts(command->opcode)) {
        Serial.println("# Metal detector command rejected: invalid action");
        return;
    }

    if (metal_detector_action_controller_is_busy(controller)) {
        Serial.println("# Metal detector command rejected: controller busy");
        return;
    }

    if (!metal_detector_driver_is_enabled(controller->detector)) {
        Serial.println("# Metal detector command rejected: controller not ready");
        queue_result(
            controller,
            STATUS_FAULT,
            static_cast<uint8_t>(command->opcode));
        return;
    }

    if (command->opcode == CMD_METAL_SET_BASELINE) {
        servo_set_angle(&rock_lift_servo, ROCK_LIFT_SEMI_LOWERED_ANGLE);
        controller->stage = METAL_ACTION_WAITING_FOR_SEMI_LOWER;
        controller->stage_complete_ms = millis() + kLiftSettleMs;
        Serial.println("# Rock arm semi-lowering before baseline");
        return;
    }

    if (!metal_detector_driver_has_baseline(controller->detector)) {
        Serial.println("# Metal detector read rejected: baseline unavailable");
        queue_result(controller, STATUS_FAULT, CMD_METAL_READ);
        return;
    }

    servo_set_position(&rock_lift_servo, SERVO_POSITION_A);
    controller->stage = METAL_ACTION_WAITING_FOR_LOWER;
    controller->stage_complete_ms = millis() + kLiftSettleMs;
    Serial.println("# Rock arm lowering to sample");
}

bool metal_detector_action_controller_update(
    MetalDetectorActionController *controller,
    uint32_t now_ms) {
    if (controller == nullptr) return false;

    if (controller->stage == METAL_ACTION_WAITING_FOR_SEMI_LOWER &&
        deadline_reached(now_ms, controller->stage_complete_ms)) {
        servo_set_position(&rock_claw_servo, SERVO_POSITION_A);
        controller->stage = METAL_ACTION_WAITING_FOR_CLAW_OPEN;
        controller->stage_complete_ms = now_ms + kClawSettleMs;
        Serial.println("# Rock claw opening");
    } else if (controller->stage == METAL_ACTION_WAITING_FOR_CLAW_OPEN &&
               deadline_reached(now_ms, controller->stage_complete_ms)) {
        if (begin_sample(controller)) {
            controller->stage = METAL_ACTION_SAMPLING_BASELINE;
            Serial.println("# Metal detector setting baseline");
        } else {
            start_empty_claw_retract(
                controller,
                now_ms,
                STATUS_FAULT,
                CMD_METAL_SET_BASELINE);
        }
    } else if (controller->stage == METAL_ACTION_WAITING_FOR_LOWER &&
               deadline_reached(now_ms, controller->stage_complete_ms)) {
        if (begin_sample(controller)) {
            controller->stage = METAL_ACTION_SAMPLING_ROCK;
            Serial.println("# Metal detector sampling rock");
        } else {
            start_empty_claw_retract(
                controller,
                now_ms,
                STATUS_FAULT,
                CMD_METAL_READ);
        }
    } else if (controller->stage == METAL_ACTION_WAITING_FOR_CLEARANCE &&
               deadline_reached(now_ms, controller->stage_complete_ms)) {
        servo_set_position(&rock_claw_servo, SERVO_POSITION_B);
        controller->stage = METAL_ACTION_WAITING_FOR_CLAW_CLOSE;
        controller->stage_complete_ms = now_ms + kClawSettleMs;
        Serial.println("# Closing rock claw above pickup position");
    } else if (controller->stage == METAL_ACTION_WAITING_FOR_CLAW_CLOSE &&
               deadline_reached(now_ms, controller->stage_complete_ms)) {
        servo_set_position(&rock_lift_servo, SERVO_POSITION_B);
        controller->stage = METAL_ACTION_WAITING_FOR_FULL_LIFT;
        controller->stage_complete_ms = now_ms + kLiftSettleMs;
        Serial.println("# Rock arm lifting fully");
    } else if (controller->stage == METAL_ACTION_WAITING_FOR_FULL_LIFT &&
               deadline_reached(now_ms, controller->stage_complete_ms)) {
        queue_result(
            controller,
            controller->result_code,
            controller->result_detail);
    }

    if (controller->stage == METAL_ACTION_SAMPLING_BASELINE ||
        controller->stage == METAL_ACTION_SAMPLING_ROCK) {
        const bool setting_baseline =
            controller->stage == METAL_ACTION_SAMPLING_BASELINE;
        MetalDetectorSample sample = {};
        esp_err_t error = metal_detector_driver_poll_sample(
            controller->detector, &sample);
        if (error != ESP_ERR_NOT_FINISHED) {
            if (error == ESP_OK && setting_baseline) {
                error = metal_detector_driver_set_baseline(
                    controller->detector, &sample);
            }

            if (error != ESP_OK) {
                Serial.printf(
                    "# Metal detector sample failed (%s)\n",
                    esp_err_to_name(error));
                start_empty_claw_retract(
                    controller,
                    now_ms,
                    STATUS_FAULT,
                    setting_baseline ? CMD_METAL_SET_BASELINE : CMD_METAL_READ);
            } else if (setting_baseline) {
                queue_result(
                    controller,
                    STATUS_ACTION_COMPLETE,
                    STATUS_DETAIL_METAL_BASELINE_SET);
            } else {
                controller->result_code = STATUS_ACTION_COMPLETE;
                controller->result_detail = sample.detected
                    ? STATUS_DETAIL_METAL_DETECTED
                    : STATUS_DETAIL_METAL_NOT_DETECTED;
                if (sample.detected) {
                    servo_set_position(&rock_claw_servo, SERVO_POSITION_B);
                    controller->stage = METAL_ACTION_WAITING_FOR_CLAW_CLOSE;
                    controller->stage_complete_ms = now_ms + kClawSettleMs;
                    Serial.println("# Metal detected; closing rock claw");
                } else {
                    start_empty_claw_retract(
                        controller,
                        now_ms,
                        STATUS_ACTION_COMPLETE,
                        STATUS_DETAIL_METAL_NOT_DETECTED);
                    Serial.println("# No metal detected; lifting clear of rock");
                }
            }
        }
    }

    if (!controller->report_pending) return false;

    const StatusPacket status = {
        .code = controller->result_code,
        .detail = controller->result_detail,
    };
    if (status_packet_send(controller->drivetrain_uart, &status) != ESP_OK) {
        return true;
    }

    controller->report_pending = false;
    return true;
}
