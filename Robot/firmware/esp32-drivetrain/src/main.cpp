/* Coordinates the Tower placement sequence while keeping the drivetrain inert. */
#include <Arduino.h>

#include <robot_common/command_packet.h>
#include <robot_common/status_packet.h>
#include <robot_common/uart_link.h>

#include "config/communication/uart_link_config.h"
#include "config/drivetrain/drivetrain_config.h"
#include "control/drivetrain/drivetrain.h"

namespace {

constexpr uint32_t kArmActionTimeoutMs = 15000;

enum class DemoState {
    RUNNING,
    COMPLETE,
    FAULT,
};

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

constexpr size_t kTowerSequenceLength =
    sizeof(kTowerSequence) / sizeof(kTowerSequence[0]);

UartLink arm_uart = {};
DemoState demo_state = DemoState::FAULT;
size_t current_step = 0;
uint32_t arm_action_deadline_ms = 0;

// Signed subtraction keeps this comparison valid across millis() wraparound.
bool deadline_reached(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

void enter_fault(const char *reason, esp_err_t error = ESP_FAIL) {
    demo_state = DemoState::FAULT;
    Serial.printf("# DEMO FAULT: %s (%s)\n", reason, esp_err_to_name(error));
}

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

esp_err_t start_tower_step(size_t step_index) {
    const TowerSequenceStep &step = kTowerSequence[step_index];
    const CommandPacket command = {
        .opcode = step.command,
        .value = step.command_value,
    };
    const esp_err_t error = command_packet_send(&arm_uart, &command);
    if (error == ESP_OK) {
        demo_state = DemoState::RUNNING;
        arm_action_deadline_ms = millis() + kArmActionTimeoutMs;
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
    }
    return error;
}

void finish_current_step() {
    const TowerSequenceStep &completed_step = kTowerSequence[current_step];
    Serial.printf(
        "# Completed task %u/%u: %s\n",
        static_cast<unsigned>(current_step + 1),
        static_cast<unsigned>(kTowerSequenceLength),
        tower_command_name(completed_step.command));

    ++current_step;
    if (current_step >= kTowerSequenceLength) {
        demo_state = DemoState::COMPLETE;
        Serial.println("# Tower action sequence complete");
        return;
    }

    Serial.println("# Tower idle");
    const esp_err_t error = start_tower_step(current_step);
    if (error != ESP_OK) {
        enter_fault("failed to start next Tower action", error);
    }
}

void service_arm_uart() {
    const esp_err_t update_error = uart_link_update(&arm_uart);
    if (update_error != ESP_OK) {
        enter_fault("arm UART update failed", update_error);
        return;
    }

    PacketFrame frame = {};
    if (uart_link_take_packet(&arm_uart, &frame) != ESP_OK ||
        !status_packet_is(&frame)) {
        return;
    }

    StatusPacket status = {};
    if (status_packet_decode(&frame, &status) != ESP_OK) return;
    if (status.code == STATUS_FAULT) {
        enter_fault("arm reported a fault");
        return;
    }

    if (status.code == STATUS_ACTION_COMPLETE &&
        status.detail == static_cast<uint8_t>(
            kTowerSequence[current_step].completion_detail)) {
        finish_current_step();
    }
}

}  // namespace

void setup() {
    // No drivetrain lifecycle is started for this Tower-only hardware test.
    (void)drivetrain_hold_safe_outputs(&DRIVETRAIN_CONFIG);
    Serial.begin(115200);
    delay(5000);
    Serial.println("# Starting Tower-only action sequence");

    const esp_err_t uart_error =
        uart_link_init(&arm_uart, &TOP_ESP_UART_LINK_CONFIG);
    if (uart_error != ESP_OK) {
        enter_fault("arm UART initialization failed", uart_error);
        return;
    }

    current_step = 0;
    const esp_err_t action_error = start_tower_step(current_step);
    if (action_error != ESP_OK) {
        enter_fault("failed to start Tower sequence", action_error);
    }
}

void loop() {
    if (demo_state == DemoState::FAULT ||
        demo_state == DemoState::COMPLETE) {
        delay(10);
        return;
    }

    service_arm_uart();
    if (demo_state == DemoState::FAULT ||
        demo_state == DemoState::COMPLETE) {
        return;
    }

    if (deadline_reached(millis(), arm_action_deadline_ms)) {
        enter_fault("arm action timed out", ESP_ERR_TIMEOUT);
        return;
    }

    delay(1);
}
