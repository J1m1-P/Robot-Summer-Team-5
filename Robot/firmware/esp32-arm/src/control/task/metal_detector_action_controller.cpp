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

// Handles millis() rollover while waiting for the current motion to settle.
bool stage_has_settled(
    const MetalDetectorActionController *controller,
    uint32_t now_ms) {
    return static_cast<int32_t>(
        now_ms - controller->stage_complete_ms) >= 0;
}

void store_result(
    MetalDetectorActionController *controller,
    StatusCode code,
    uint8_t detail) {
    controller->result_code = code;
    controller->result_detail = detail;
}

void wait_for_stage(
    MetalDetectorActionController *controller,
    MetalDetectorActionStage stage,
    uint32_t now_ms,
    uint32_t settle_ms) {
    controller->stage = stage;
    controller->stage_complete_ms = now_ms + settle_ms;
}

// Retains a result until the shared UART can accept it.
void queue_result(
    MetalDetectorActionController *controller,
    StatusCode code,
    uint8_t detail) {
    controller->stage = METAL_ACTION_IDLE;
    store_result(controller, code, detail);
    controller->report_pending = true;
}

// Starts a fresh detector window and enters its polling stage.
bool start_sample(
    MetalDetectorActionController *controller,
    MetalDetectorActionStage sampling_stage,
    const char *start_message) {
    const esp_err_t error =
        metal_detector_driver_begin_sample(controller->detector);
    if (error == ESP_OK) {
        controller->stage = sampling_stage;
        Serial.println(start_message);
        return true;
    }

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
    store_result(controller, result_code, result_detail);
    servo_set_angle(&rock_lift_servo, ROCK_LIFT_SEMI_LOWERED_ANGLE);
    wait_for_stage(
        controller,
        METAL_ACTION_WAITING_FOR_CLEARANCE,
        now_ms,
        kLiftSettleMs);
}

// Releases a centered rock at ground level before the empty-claw retract.
void start_ground_release(
    MetalDetectorActionController *controller,
    uint32_t now_ms,
    StatusCode result_code,
    uint8_t result_detail) {
    store_result(controller, result_code, result_detail);
    servo_set_position(&rock_claw_servo, SERVO_POSITION_A);
    wait_for_stage(
        controller,
        METAL_ACTION_WAITING_FOR_GROUND_RELEASE,
        now_ms,
        kClawSettleMs);
}

// Polls either sample type and starts the matching completion/recovery path.
void update_sample(
    MetalDetectorActionController *controller,
    uint32_t now_ms,
    bool setting_baseline) {
    MetalDetectorSample sample = {};
    esp_err_t error = metal_detector_driver_poll_sample(
        controller->detector, &sample);
    if (error == ESP_ERR_NOT_FINISHED) return;

    if (error == ESP_OK && setting_baseline) {
        error = metal_detector_driver_set_baseline(
            controller->detector, &sample);
    }

    if (error != ESP_OK) {
        Serial.printf(
            "# Metal detector sample failed (%s)\n",
            esp_err_to_name(error));
        if (setting_baseline) {
            start_empty_claw_retract(
                controller,
                now_ms,
                STATUS_FAULT,
                CMD_METAL_SET_BASELINE);
        } else {
            start_ground_release(
                controller,
                now_ms,
                STATUS_FAULT,
                CMD_METAL_READ);
        }
        return;
    }

    if (setting_baseline) {
        queue_result(
            controller,
            STATUS_ACTION_COMPLETE,
            STATUS_DETAIL_METAL_BASELINE_SET);
        return;
    }

    store_result(
        controller,
        STATUS_ACTION_COMPLETE,
        sample.detected
            ? STATUS_DETAIL_METAL_DETECTED
            : STATUS_DETAIL_METAL_NOT_DETECTED);
    if (sample.detected) {
        servo_set_position(&rock_lift_servo, SERVO_POSITION_B);
        wait_for_stage(
            controller,
            METAL_ACTION_WAITING_FOR_FULL_LIFT,
            now_ms,
            kLiftSettleMs);
        Serial.println("# Metal detected; lifting centered rock");
        return;
    }

    start_ground_release(
        controller,
        now_ms,
        STATUS_ACTION_COMPLETE,
        STATUS_DETAIL_METAL_NOT_DETECTED);
    Serial.println("# No metal detected; opening claw");
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
        wait_for_stage(
            controller,
            METAL_ACTION_WAITING_FOR_SEMI_LOWER,
            millis(),
            kLiftSettleMs);
        Serial.println("# Rock arm semi-lowering before baseline");
        return;
    }

    if (!metal_detector_driver_has_baseline(controller->detector)) {
        Serial.println("# Metal detector read rejected: baseline unavailable");
        queue_result(controller, STATUS_FAULT, CMD_METAL_READ);
        return;
    }

    servo_set_position(&rock_lift_servo, SERVO_POSITION_A);
    wait_for_stage(
        controller,
        METAL_ACTION_WAITING_FOR_LOWER,
        millis(),
        kLiftSettleMs);
    Serial.println("# Rock arm lowering to sample");
}

bool metal_detector_action_controller_update(
    MetalDetectorActionController *controller,
    uint32_t now_ms) {
    if (controller == nullptr) return false;

    switch (controller->stage) {
        case METAL_ACTION_IDLE:
            break;

        case METAL_ACTION_WAITING_FOR_SEMI_LOWER:
            if (!stage_has_settled(controller, now_ms)) break;
            servo_set_position(&rock_claw_servo, SERVO_POSITION_A);
            wait_for_stage(
                controller,
                METAL_ACTION_WAITING_FOR_CLAW_OPEN,
                now_ms,
                kClawSettleMs);
            Serial.println("# Rock claw opening");
            break;

        case METAL_ACTION_WAITING_FOR_CLAW_OPEN:
            if (!stage_has_settled(controller, now_ms)) break;
            if (!start_sample(
                    controller,
                    METAL_ACTION_SAMPLING_BASELINE,
                    "# Metal detector setting baseline")) {
                start_empty_claw_retract(
                    controller,
                    now_ms,
                    STATUS_FAULT,
                    CMD_METAL_SET_BASELINE);
            }
            break;

        case METAL_ACTION_SAMPLING_BASELINE:
            update_sample(controller, now_ms, true);
            break;

        case METAL_ACTION_WAITING_FOR_LOWER:
            if (!stage_has_settled(controller, now_ms)) break;
            servo_set_position(&rock_claw_servo, SERVO_POSITION_B);
            wait_for_stage(
                controller,
                METAL_ACTION_WAITING_FOR_CENTERING_CLAW_CLOSE,
                now_ms,
                kClawSettleMs);
            Serial.println("# Rock claw closing to center rock");
            break;

        case METAL_ACTION_WAITING_FOR_CENTERING_CLAW_CLOSE:
            if (!stage_has_settled(controller, now_ms)) break;
            if (!start_sample(
                    controller,
                    METAL_ACTION_SAMPLING_ROCK,
                    "# Metal detector sampling centered rock")) {
                start_ground_release(
                    controller,
                    now_ms,
                    STATUS_FAULT,
                    CMD_METAL_READ);
            }
            break;

        case METAL_ACTION_SAMPLING_ROCK:
            update_sample(controller, now_ms, false);
            break;

        case METAL_ACTION_WAITING_FOR_GROUND_RELEASE:
            if (!stage_has_settled(controller, now_ms)) break;
            start_empty_claw_retract(
                controller,
                now_ms,
                controller->result_code,
                controller->result_detail);
            Serial.println("# Rock released; lifting to semi-lowered state");
            break;

        case METAL_ACTION_WAITING_FOR_CLEARANCE:
            if (!stage_has_settled(controller, now_ms)) break;
            servo_set_position(&rock_claw_servo, SERVO_POSITION_B);
            wait_for_stage(
                controller,
                METAL_ACTION_WAITING_FOR_CLAW_CLOSE,
                now_ms,
                kClawSettleMs);
            Serial.println("# Closing rock claw at semi-lowered state");
            break;

        case METAL_ACTION_WAITING_FOR_CLAW_CLOSE:
            if (!stage_has_settled(controller, now_ms)) break;
            servo_set_position(&rock_lift_servo, SERVO_POSITION_B);
            wait_for_stage(
                controller,
                METAL_ACTION_WAITING_FOR_FULL_LIFT,
                now_ms,
                kLiftSettleMs);
            Serial.println("# Rock arm lifting fully");
            break;

        case METAL_ACTION_WAITING_FOR_FULL_LIFT:
            if (!stage_has_settled(controller, now_ms)) break;
            if (controller->result_detail == STATUS_DETAIL_METAL_DETECTED) {
                Serial.println("# Metal rock lifted; keeping claw closed");
            }
            queue_result(
                controller,
                controller->result_code,
                controller->result_detail);
            break;
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
