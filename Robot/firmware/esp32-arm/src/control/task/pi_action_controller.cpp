/* Implements the non-blocking Raspberry Pi request/report transaction. */
#include "control/task/pi_action_controller.h"

#include <Arduino.h>

namespace {

// Bounds one Pi request so a missing response cannot stall the sequence.
constexpr uint32_t kResponseTimeoutMs = 15000;

// Signed subtraction keeps deadline checks valid across millis() wraparound.
bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms) {
    return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

// Completes the active request and queues one report for the drivetrain.
void finish_action(
    PiActionController *controller,
    const PiReportPacket &report) {
    controller->action_active = false;
    (void)pi_report_packet_send(controller->drivetrain_uart, &report);
}

// Converts a local UART or timeout failure into a normal Pi report so the
// drivetrain receives one result format for both successful and failed scans.
void finish_with_error(
    PiActionController *controller,
    PiResultCode result) {
    const PiReportPacket report = {
        .request_id = controller->request_id,
        .action = PI_ACTION_SCAN_TELETUBBIES,
        .result = result,
        .target_id = 0U,
        .horizontal_error = 0.0f,
        .confidence_percent = 0U,
    };
    finish_action(controller, report);
}

}  // namespace

// Called during setup(). Connects the controller to both initialized links.
void pi_action_controller_init(
    PiActionController *controller,
    UartLink *pi_uart,
    UartLink *drivetrain_uart) {
    *controller = {};
    controller->pi_uart = pi_uart;
    controller->drivetrain_uart = drivetrain_uart;
}

// Identifies the command owned by this controller.
bool pi_action_controller_accepts(CommandOpcode command) {
    return command == CMD_PI_SCAN_TELETUBBIES;
}

// The Pi action remains busy until its matching report or a local failure.
bool pi_action_controller_is_busy(const PiActionController *controller) {
    return controller->action_active;
}

// Starts one scan request and records the response deadline.
void pi_action_controller_start(
    PiActionController *controller,
    const CommandPacket *command,
    uint32_t now_ms) {
    ++controller->request_id;

    const PiRequestPacket request = {
        .request_id = controller->request_id,
        .action = PI_ACTION_SCAN_TELETUBBIES,
        .parameter = command->value,
    };

    const esp_err_t error =
        pi_request_packet_send(controller->pi_uart, &request);
    if (error != ESP_OK) {
        Serial.printf(
            "# Pi request send failed (%s)\n",
            esp_err_to_name(error));
        finish_with_error(controller, PI_RESULT_LINK_ERROR);
        return;
    }

    controller->action_active = true;
    controller->response_deadline_ms = now_ms + kResponseTimeoutMs;
}

bool pi_action_controller_update(
    PiActionController *controller,
    uint32_t now_ms) {
    bool report_sent = false;

    // This controller is the only reader of the dedicated Pi UART.
    const esp_err_t update_error = uart_link_update(controller->pi_uart);

    if (update_error != ESP_OK && controller->action_active) {
        Serial.printf(
            "# Pi UART update failed (%s)\n",
            esp_err_to_name(update_error));
        finish_with_error(controller, PI_RESULT_LINK_ERROR);
        report_sent = true;
    } else if (controller->action_active) {
        // Consume at most one complete report per update.
        PacketFrame frame = {};
        if (uart_link_take_packet(controller->pi_uart, &frame) == ESP_OK) {
            PiReportPacket report = {};

            // Only the report for the active request can finish this action.
            if (pi_report_packet_decode(&frame, &report) == ESP_OK &&
                report.request_id == controller->request_id) {
                finish_action(controller, report);
                report_sent = true;
            }
        }
    }

    // Convert a missing Pi response into a timeout report for drivetrain.
    if (controller->action_active &&
        deadline_reached(now_ms, controller->response_deadline_ms)) {
        Serial.println("# Pi action timed out");
        finish_with_error(controller, PI_RESULT_TIMEOUT);
        report_sent = true;
    }

    return report_sent;
}
