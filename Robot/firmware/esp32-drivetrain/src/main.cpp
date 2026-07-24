/* Runs the Tower-only action sequence while keeping the drivetrain inert. */
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
    ActionStatusDetail completion;
    const char *start_message;
};

// Reorder these independent action blocks to change the test sequence.
constexpr TowerSequenceStep kTowerSequence[] = {
    {CMD_TOWER_Z_UP, STATUS_DETAIL_TOWER_Z_RAISED,
     "# Raising Tower 0.1 m"},
    {CMD_TOWER_Z_DOWN, STATUS_DETAIL_TOWER_Z_LOWERED,
     "# Lowering Tower 0.1 m"},
    {CMD_TOWER_X_LEFT, STATUS_DETAIL_TOWER_X_LEFT,
     "# Moving Tower left 0.05 m"},
    {CMD_TOWER_X_RIGHT, STATUS_DETAIL_TOWER_X_RIGHT,
     "# Moving Tower right 0.05 m"},
};

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

esp_err_t start_tower_step(size_t step_index) {
    const TowerSequenceStep &step = kTowerSequence[step_index];
    const CommandPacket command = {
        .opcode = step.command,
        .value = 0.0f,
    };
    const esp_err_t error = command_packet_send(&arm_uart, &command);
    if (error == ESP_OK) {
        demo_state = DemoState::RUNNING;
        arm_action_deadline_ms = millis() + kArmActionTimeoutMs;
        Serial.println(step.start_message);
    }
    return error;
}

void finish_current_step() {
    ++current_step;
    if (current_step >=
        sizeof(kTowerSequence) / sizeof(kTowerSequence[0])) {
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

    const ActionStatusDetail detail =
        static_cast<ActionStatusDetail>(status.detail);
    if (status.code == STATUS_ACTION_COMPLETE &&
        detail == kTowerSequence[current_step].completion) {
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
        enter_fault("failed to start Tower raise", action_error);
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
