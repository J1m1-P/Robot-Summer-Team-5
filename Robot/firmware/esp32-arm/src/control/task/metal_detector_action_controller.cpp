/* Implements synchronous metal detector baseline-set and read commands. */
#include "control/task/metal_detector_action_controller.h"

#include <Arduino.h>

#include <robot_common/status_packet.h>

namespace {

// Status transmission is intentionally best effort, matching the other
// arm action controllers.
void send_status(UartLink *drivetrain_uart, StatusCode code, uint8_t detail) {
    const StatusPacket status = {
        .code = code,
        .detail = detail,
    };
    (void)status_packet_send(drivetrain_uart, &status);
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

// Both actions complete synchronously in start(); never blocks a follow-up.
bool metal_detector_action_controller_is_busy(
    const MetalDetectorActionController *controller) {
    (void)controller;
    return false;
}

// Executes one command immediately against the local pulse-counter driver.
void metal_detector_action_controller_start(
    MetalDetectorActionController *controller,
    const CommandPacket *command) {

    if (!metal_detector_driver_is_enabled(controller->detector)) {
        Serial.println("# Metal detector command rejected: detector not ready");
        send_status(
            controller->drivetrain_uart,
            STATUS_FAULT,
            static_cast<uint8_t>(command->opcode));
        return;
    }

    if (command->opcode == CMD_METAL_SET_BASELINE) {
        const int16_t count = metal_detector_driver_get_count(controller->detector);
        metal_detector_driver_set_comparison_count(controller->detector, count);
        controller->pending_detail = static_cast<uint8_t>(STATUS_DETAIL_NONE);
    } else {
        const bool detected = metal_detector_driver_read(controller->detector);
        controller->pending_detail = detected
            ? static_cast<uint8_t>(STATUS_DETAIL_METAL_DETECTED)
            : static_cast<uint8_t>(STATUS_DETAIL_NONE);
    }

    controller->report_pending = true;
}

// Keeps the detector's sample window paced and reports one queued status.
bool metal_detector_action_controller_update(
    MetalDetectorActionController *controller,
    uint32_t now_ms) {
    (void)now_ms;

    // Must run every tick regardless of pending commands so the driver's
    // fixed sample window stays current between reads.
    (void)metal_detector_driver_read(controller->detector);

    if (!controller->report_pending) return false;

    controller->report_pending = false;
    send_status(
        controller->drivetrain_uart,
        STATUS_ACTION_COMPLETE,
        controller->pending_detail);
    return true;
}
