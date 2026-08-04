/* Implements fresh-window metal detector baseline-set and read commands. */
#include "control/task/metal_detector_action_controller.h"

#include <Arduino.h>

#include <robot_common/status_packet.h>

namespace {

esp_err_t send_status(
    UartLink *drivetrain_uart,
    StatusCode code,
    uint8_t detail) {

    const StatusPacket status = {
        .code = code,
        .detail = detail,
    };
    return status_packet_send(drivetrain_uart, &status);
}

void queue_status(
    MetalDetectorActionController *controller,
    StatusCode code,
    uint8_t detail) {

    controller->pending_code = code;
    controller->pending_detail = detail;
    controller->report_pending = true;
}

}  // namespace

// Called during setup(). Connects the controller to its UART and driver.
void metal_detector_action_controller_init(
    MetalDetectorActionController *controller,
    UartLink *drivetrain_uart,
    MetalDetectorDriver *detector) {
    *controller = {};
    controller->drivetrain_uart = drivetrain_uart;
    controller->detector = detector;
}

// Identifies the commands owned by this controller.
bool metal_detector_action_controller_accepts(CommandOpcode command) {
    return command == CMD_METAL_SET_BASELINE || command == CMD_METAL_READ;
}

bool metal_detector_action_controller_is_busy(
    const MetalDetectorActionController *controller) {
    return controller != nullptr &&
        (controller->action_active || controller->report_pending);
}

// Starts a command-owned sample so the result cannot come from an old pose.
void metal_detector_action_controller_start(
    MetalDetectorActionController *controller,
    const CommandPacket *command) {

    if (controller == nullptr || command == nullptr) return;

    if (!metal_detector_action_controller_accepts(command->opcode) ||
        !metal_detector_driver_is_enabled(controller->detector)) {
        Serial.println("# Metal detector command rejected: detector not ready");
        queue_status(
            controller,
            STATUS_FAULT,
            static_cast<uint8_t>(command->opcode));
        return;
    }

    const esp_err_t sample_error =
        metal_detector_driver_begin_sample(controller->detector);
    if (sample_error != ESP_OK) {
        Serial.printf(
            "# Metal detector sample start failed (%s)\n",
            esp_err_to_name(sample_error));
        queue_status(
            controller,
            STATUS_FAULT,
            static_cast<uint8_t>(command->opcode));
        return;
    }

    controller->active_command = command->opcode;
    controller->action_active = true;
}

// Completes the active sample, then retains its status until UART accepts it.
bool metal_detector_action_controller_update(
    MetalDetectorActionController *controller,
    uint32_t now_ms) {
    (void)now_ms;

    if (controller == nullptr) return false;

    if (controller->action_active) {
        MetalDetectorSample sample = {};
        esp_err_t sample_error = metal_detector_driver_poll_sample(
            controller->detector, &sample);
        if (sample_error == ESP_ERR_NOT_FINISHED) return false;

        controller->action_active = false;
        if (sample_error != ESP_OK) {
            Serial.printf(
                "# Metal detector sample failed (%s)\n",
                esp_err_to_name(sample_error));
            queue_status(
                controller,
                STATUS_FAULT,
                static_cast<uint8_t>(controller->active_command));
        } else if (controller->active_command == CMD_METAL_SET_BASELINE) {
            sample_error = metal_detector_driver_set_baseline(
                controller->detector, &sample);
            if (sample_error == ESP_OK) {
                queue_status(
                    controller,
                    STATUS_ACTION_COMPLETE,
                    static_cast<uint8_t>(STATUS_DETAIL_METAL_BASELINE_SET));
            } else {
                queue_status(
                    controller,
                    STATUS_FAULT,
                    static_cast<uint8_t>(controller->active_command));
            }
        } else if (!sample.baseline_valid) {
            Serial.println("# Metal detector read failed: baseline unavailable");
            queue_status(
                controller,
                STATUS_FAULT,
                static_cast<uint8_t>(controller->active_command));
        } else {
            queue_status(
                controller,
                STATUS_ACTION_COMPLETE,
                static_cast<uint8_t>(sample.detected
                    ? STATUS_DETAIL_METAL_DETECTED
                    : STATUS_DETAIL_METAL_NOT_DETECTED));
        }
    }

    if (!controller->report_pending) return false;

    const esp_err_t send_error = send_status(
        controller->drivetrain_uart,
        controller->pending_code,
        controller->pending_detail);
    if (send_error != ESP_OK) {
        // Keep lower-priority odometry off the shared UART until this result
        // can be retried on the next dispatcher tick.
        return true;
    }

    controller->report_pending = false;
    return true;
}
