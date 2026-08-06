/* Implements the staged rock sampling and pickup workflow. */
#include "control/task/metal_detector_action_controller.h"

#include <Arduino.h>

#include "config/servo_config.h"
#include "drivers/servo_driver.h"

namespace {

// Required delay between staged lift and claw movements.
constexpr uint32_t kServoDelayMs = 200;

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

// Moves to down, then closes the claw before the later up movement.
void start_down_claw_close(
    MetalDetectorActionController *controller,
    uint32_t now_ms,
    StatusCode result_code,
    uint8_t result_detail) {
    store_result(controller, result_code, result_detail);
    servo_set_position(&rock_lift_servo, SERVO_POSITION_A);
    wait_for_stage(
        controller,
        METAL_ACTION_WAITING_FOR_DOWN_POSITION,
        now_ms,
        kServoDelayMs);
}

// Ensures the claw is closed and settled before moving the lift up.
void start_up_after_claw_close(
    MetalDetectorActionController *controller,
    uint32_t now_ms,
    StatusCode result_code,
    uint8_t result_detail) {
    store_result(controller, result_code, result_detail);
    servo_set_position(&rock_claw_servo, SERVO_POSITION_B);
    wait_for_stage(
        controller,
        METAL_ACTION_WAITING_FOR_UP_CLAW_CLOSE,
        now_ms,
        kServoDelayMs);
}

// Polls the active detector sample and starts its next state.
void update_sample(
    MetalDetectorActionController *controller,
    uint32_t now_ms) {
    MetalDetectorSample sample = {};
    esp_err_t error = metal_detector_driver_poll_sample(
        controller->detector, &sample);
    if (error == ESP_ERR_NOT_FINISHED) return;

    if (error == ESP_OK &&
        controller->stage == METAL_ACTION_SAMPLING_BASELINE) {
        error = metal_detector_driver_set_baseline(
            controller->detector, &sample);
    }

    if (error != ESP_OK) {
        Serial.printf(
            "# Metal detector sample failed (%s)\n",
            esp_err_to_name(error));
        start_down_claw_close(
            controller,
            now_ms,
            STATUS_FAULT,
            CMD_METAL_READ);
        return;
    }

    if (controller->stage == METAL_ACTION_SAMPLING_BASELINE) {
        servo_set_position(&rock_claw_servo, SERVO_POSITION_B);
        wait_for_stage(
            controller,
            METAL_ACTION_WAITING_FOR_BASELINE_CLAW_CLOSE,
            now_ms,
        kServoDelayMs);
        Serial.println("# Baseline set; closing claw before read position");
        return;
    }

    store_result(
        controller,
        STATUS_ACTION_COMPLETE,
        sample.detected
            ? STATUS_DETAIL_METAL_DETECTED
            : STATUS_DETAIL_METAL_NOT_DETECTED);
    if (sample.detected) {
        controller->positive_detection_made = true;
        servo_set_position(&rock_claw_servo, SERVO_POSITION_B);
        wait_for_stage(
            controller,
            METAL_ACTION_WAITING_FOR_UP_CLAW_CLOSE,
            now_ms,
            kServoDelayMs);
        Serial.println("# Metal detected; closing claw before lifting centered rock");
        return;
    }

    start_down_claw_close(
        controller,
        now_ms,
        STATUS_ACTION_COMPLETE,
        STATUS_DETAIL_METAL_NOT_DETECTED);
    Serial.println("# No metal detected; moving down before closing claw and lifting");
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
    servo_set_position(&rock_claw_servo, SERVO_POSITION_B);
    servo_set_position(&rock_lift_servo, SERVO_POSITION_B);
}

bool metal_detector_action_controller_accepts(CommandOpcode command) {
    return command == CMD_METAL_READ;
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

    if (controller->positive_detection_made) {
        queue_result(
            controller,
            STATUS_ACTION_COMPLETE,
            STATUS_DETAIL_METAL_DETECTED);
        Serial.println("# Positive rock detection already made; skipping claw workflow");
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

    // Every read establishes a fresh baseline from the open claw at down.
    servo_set_position(&rock_lift_servo, SERVO_POSITION_A);
    wait_for_stage(
        controller,
        METAL_ACTION_WAITING_FOR_ARM_LOWER,
        millis(),
        kServoDelayMs);
    Serial.println("# Rock arm lowering to sample");
}

bool metal_detector_action_controller_update(
    MetalDetectorActionController *controller,
    uint32_t now_ms) {
    if (controller == nullptr) return false;

    switch (controller->stage) {
        case METAL_ACTION_IDLE:
            break;

        case METAL_ACTION_WAITING_FOR_ARM_LOWER:
            if (!stage_has_settled(controller, now_ms)) break;
            servo_set_position(&rock_claw_servo, SERVO_POSITION_A);
            wait_for_stage(
                controller,
                METAL_ACTION_WAITING_FOR_ARM_CLAW_OPEN,
                now_ms,
                kServoDelayMs);
            Serial.println("# Rock claw opening");
            break;

        case METAL_ACTION_WAITING_FOR_ARM_CLAW_OPEN:
            if (!stage_has_settled(controller, now_ms)) break;
            if (!start_sample(
                    controller,
                    METAL_ACTION_SAMPLING_BASELINE,
                    "# Metal detector setting baseline")) {
                start_down_claw_close(
                    controller,
                    now_ms,
                    STATUS_FAULT,
                    CMD_METAL_READ);
            }
            break;

        case METAL_ACTION_SAMPLING_BASELINE:
            update_sample(controller, now_ms);
            break;

        case METAL_ACTION_WAITING_FOR_BASELINE_CLAW_CLOSE:
            if (!stage_has_settled(controller, now_ms)) break;
            servo_set_angle(&rock_lift_servo, ROCK_LIFT_READ_ANGLE);
            wait_for_stage(
                controller,
                METAL_ACTION_WAITING_FOR_READ_POSITION,
                now_ms,
                kServoDelayMs);
            Serial.println("# Rock claw closed; moving arm to read position");
            break;

        case METAL_ACTION_WAITING_FOR_READ_POSITION:
            if (!stage_has_settled(controller, now_ms)) break;
            if (!start_sample(
                    controller,
                    METAL_ACTION_SAMPLING_ROCK,
                    "# Metal detector sampling at read position")) {
                start_down_claw_close(
                    controller,
                    now_ms,
                    STATUS_FAULT,
                    CMD_METAL_READ);
            }
            break;

        case METAL_ACTION_SAMPLING_ROCK:
            update_sample(controller, now_ms);
            break;

        case METAL_ACTION_WAITING_FOR_DOWN_POSITION:
            if (!stage_has_settled(controller, now_ms)) break;
            start_up_after_claw_close(
                controller,
                now_ms,
                controller->result_code,
                controller->result_detail);
            Serial.println("# Rock absent; closing claw at down position");
            break;

        case METAL_ACTION_WAITING_FOR_UP_CLAW_CLOSE:
            if (!stage_has_settled(controller, now_ms)) break;
            servo_set_position(&rock_lift_servo, SERVO_POSITION_B);
            wait_for_stage(
                controller,
                METAL_ACTION_WAITING_FOR_FULL_LIFT,
                now_ms,
                kServoDelayMs);
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
