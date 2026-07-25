/* Implements the drivetrain-side Tower demo sequence and status handling. */
#include "control/tower_sequence_controller.h"

#include <Arduino.h>

#include <robot_common/command_packet.h>
#include <robot_common/status_packet.h>

namespace {

constexpr uint32_t kArmActionTimeoutMs = 15000;

// Describes one command and the completion status expected from the arm.
struct TowerSequenceStep {
    CommandOpcode command;
    float command_value;
    ActionStatusDetail completion_detail;
    const char *start_message;
};

constexpr TowerSequenceStep kTowerSequence[] = {
    {CMD_TOWER_HOME, 0.0f,
     STATUS_DETAIL_TOWER_HOME,
     "# 1: Setting Tower home and retracting locator"},
    {CMD_TOWER_ROTATE_VERTICAL, 0.0f, STATUS_DETAIL_TOWER_VERTICAL,
     "# 2: Rotating Tower vertical"},
    {CMD_TOWER_OPEN_CLAW, 0.0f,
     STATUS_DETAIL_TOWER_CLAW_OPEN,
     "# 3: Opening all Tower claws"},
    {CMD_TOWER_Z_UP, 0.50f,
     STATUS_DETAIL_TOWER_Z_RAISED,
     "# 4: Raising Tower claw 50 mm"},
    {CMD_TOWER_ROTATE_HORIZONTAL, 0.0f, STATUS_DETAIL_TOWER_HORIZONTAL,
     "# 5: Rotating Tower horizontal"},
    {CMD_TOWER_Z_DOWN, 0.50f,
     STATUS_DETAIL_TOWER_Z_LOWERED,
     "# 6: Lowering Tower claw 50 mm to home"},
    {CMD_TOWER_CLOSE_CLAW, 0.0f,
     STATUS_DETAIL_TOWER_CLAW_CLOSED,
     "# 7: Closing all Tower claws"},
    {CMD_TOWER_Z_UP, 0.50f,
     STATUS_DETAIL_TOWER_Z_RAISED,
     "# 8: Raising Tower claw 50 mm"},
    {CMD_TOWER_ROTATE_VERTICAL, 0.0f, STATUS_DETAIL_TOWER_VERTICAL,
     "# 9: Rotating Tower vertical"},
    {CMD_TOWER_Z_UP, 0.30f,
     STATUS_DETAIL_TOWER_Z_RAISED,
     "# 10: Raising Tower claw 30 mm"},
};

static constexpr size_t kTowerSequenceLength =
    sizeof(kTowerSequence) / sizeof(kTowerSequence[0]);

// Compares millisecond deadlines safely across millis() wraparound.
static bool deadline_reached(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

// Stops the sequence and reports why it can no longer continue.
// may not need, no choice if fault detected
void enter_fault(
    TowerSequenceController *controller,
    const char *reason,
    esp_err_t error = ESP_FAIL) {
    controller->running = false;
    Serial.printf("# DEMO FAULT: %s (%s)\n", reason, esp_err_to_name(error));
}

// Returns a short diagnostic name for a Tower command.
// Probably don't need
const char *tower_command_name(CommandOpcode command) {
    switch (command) {
        case CMD_TOWER_HOME:
            return "HOME";
        case CMD_TOWER_Z_UP:
            return "Z_UP";
        case CMD_TOWER_Z_DOWN:
            return "Z_DOWN";
        case CMD_TOWER_ROTATE_VERTICAL:
            return "ROTATE_VERTICAL";
        case CMD_TOWER_ROTATE_HORIZONTAL:
            return "ROTATE_HORIZONTAL";
        case CMD_TOWER_OPEN_CLAW:
            return "OPEN_CLAW";
        case CMD_TOWER_CLOSE_CLAW:
            return "CLOSE_CLAW";
        default:
            return "UNKNOWN";
    }
}

// Sends one sequence command and starts its completion timeout.
esp_err_t start_tower_step(
    TowerSequenceController *controller,
    size_t step_index) {
    const TowerSequenceStep &step = kTowerSequence[step_index];
    const CommandPacket command = {
        .opcode = step.command,
        .value = step.command_value,
    };
    const esp_err_t error =
        command_packet_send(controller->arm_uart, &command);
    if (error != ESP_OK) return error;

    controller->running = true;
    controller->action_deadline_ms = millis() + kArmActionTimeoutMs;
    Serial.println(step.start_message);
    Serial.printf(
        "# Executing task %u/%u: %s",
        static_cast<unsigned>(step_index + 1),
        static_cast<unsigned>(kTowerSequenceLength),
        tower_command_name(step.command));
    if (step.command == CMD_TOWER_Z_UP ||
        step.command == CMD_TOWER_Z_DOWN) {
        Serial.printf(" (%.0f mm)", step.command_value * 100.0f);
    }
    Serial.println();
    return ESP_OK;
}

// Records one completion and starts the next step when one remains.
void finish_current_step(TowerSequenceController *controller) {
    const TowerSequenceStep &completed_step =
        kTowerSequence[controller->current_step];
    Serial.printf(
        "# Completed task %u/%u: %s\n",
        static_cast<unsigned>(controller->current_step + 1),
        static_cast<unsigned>(kTowerSequenceLength),
        tower_command_name(completed_step.command));

    ++controller->current_step;
    if (controller->current_step >= kTowerSequenceLength) {
        controller->running = false;
        Serial.println("# Tower action sequence complete");
        return;
    }

    Serial.println("# Tower idle");
    const esp_err_t error =
        start_tower_step(controller, controller->current_step);
    if (error != ESP_OK) {
        enter_fault(
            controller, "failed to start next Tower action", error);
    }
}

// Consumes one arm status packet and handles faults or action completion.
void service_arm_uart(TowerSequenceController *controller) {
    // Check if the packet is a fault
    const esp_err_t update_error = uart_link_update(controller->arm_uart);
    if (update_error != ESP_OK) {
        enter_fault(controller, "arm UART update failed", update_error);
        return;
    }

    PacketFrame frame = {};
    if (uart_link_take_packet(controller->arm_uart, &frame) != ESP_OK ||
        !status_packet_is(&frame)) {
        return;
    }

    StatusPacket status = {};
    if (status_packet_decode(&frame, &status) != ESP_OK) return;
    if (status.code == STATUS_FAULT) {
        enter_fault(controller, "arm reported a fault");
        return;
    }

    if (status.code == STATUS_ACTION_COMPLETE &&
        status.detail == static_cast<uint8_t>(
            kTowerSequence[controller->current_step].completion_detail)) {
        finish_current_step(controller);
    }
}

}  // namespace

esp_err_t tower_sequence_controller_init(
    TowerSequenceController *controller,
    UartLink *arm_uart) {
    if (controller == nullptr || arm_uart == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    *controller = {};
    controller->arm_uart = arm_uart;
    const esp_err_t error = start_tower_step(controller, 0);
    if (error != ESP_OK) {
        enter_fault(controller, "failed to start Tower sequence", error);
    }
    return error;
}

void tower_sequence_controller_service(
    TowerSequenceController *controller,
    uint32_t now_ms) {
    if (controller == nullptr || !controller->running) {
        delay(10);
        return;
    }

    service_arm_uart(controller);
    if (!controller->running) return;

    if (deadline_reached(now_ms, controller->action_deadline_ms)) {
        enter_fault(controller, "arm action timed out", ESP_ERR_TIMEOUT);
    }
}
