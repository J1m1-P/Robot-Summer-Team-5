/* Implements the drivetrain-side Tower demo sequence and status handling. */
#include "control/task/tower_sequence_controller.h"

#include <stdio.h>

#include "esp32-hal.h"
#include <robot_common/command_packet.h>
#include <robot_common/status_packet.h>

static const uint32_t kArmActionTimeoutMs = 15000;

// Describes one command and the completion status expected from the arm.
typedef struct {
    CommandOpcode command;
    float command_value;
} TowerSequenceStep;

// The sequence for tower building 
static const TowerSequenceStep kTowerSequence[] = {
    {CMD_TOWER_HOME, 0.0f},
    {CMD_TOWER_ROTATE_VERTICAL, 0.0f},
    {CMD_TOWER_OPEN_CLAW, 0.0f},
    {CMD_TOWER_Z_UP, 0.50f},
    {CMD_TOWER_ROTATE_HORIZONTAL, 0.0f},
    {CMD_TOWER_Z_DOWN, 0.50f},
    {CMD_TOWER_CLOSE_CLAW, 0.0f},
    {CMD_TOWER_Z_UP, 0.50f},
    {CMD_TOWER_ROTATE_VERTICAL, 0.0f},
    {CMD_TOWER_Z_UP, 0.30f},
};

// The number of steps for this sequence
static const size_t kTowerSequenceLength = sizeof(kTowerSequence) / sizeof(kTowerSequence[0]);

// Stops the sequence and reports why it can no longer continue.
static void enter_fault(
    TowerSequenceController *controller,
    const char *reason,
    esp_err_t error) {
    controller->running = false;
    printf("# DEMO FAULT: %s (%s)\n", reason, esp_err_to_name(error));
}

// Sends one sequence command and starts its completion timeout.
static esp_err_t start_tower_step(
    TowerSequenceController *controller,
    size_t step_index) {

    // Set up the currennt step and its information
    const TowerSequenceStep *step = &kTowerSequence[step_index];
    const CommandPacket command = {
        .opcode = step->command,
        .value = step->command_value,
    };

    // Send out the command to arm
    const esp_err_t error = command_packet_send(controller->arm_uart, &command);
    if (error != ESP_OK) return error;

    controller->running = true;
    controller->action_deadline_ms = millis() + kArmActionTimeoutMs;
    return ESP_OK;
}

// Consumes one arm status packet and handles faults or action completion.
static void service_arm_uart(TowerSequenceController *controller) {
    
    // Update the UART mailbox
    const esp_err_t update_error = uart_link_update(controller->arm_uart);
    if (update_error != ESP_OK) {
        enter_fault(controller, "arm UART update failed", update_error);
        return;
    }

    // Get the lastest UART message from arm 
    PacketFrame frame = {0};
    if (uart_link_take_packet(controller->arm_uart, &frame) != ESP_OK ||
        !status_packet_is(&frame)) {
        return;
    }

    // Decode the message 
    StatusPacket status = {0};
    if (status_packet_decode(&frame, &status) != ESP_OK) return;

    // Return if fault
    if (status.code == STATUS_FAULT) {
        enter_fault(controller, "arm reported a fault", ESP_FAIL);
        return;
    }

    // Execute next step if complete
    if (status.code == STATUS_ACTION_COMPLETE) {
        ++controller->current_step;
        if (controller->current_step >= kTowerSequenceLength) {
            controller->running = false;
            printf("# Tower action sequence complete\n");
            return;
        }

        const esp_err_t error =
            start_tower_step(controller, controller->current_step);
        if (error != ESP_OK) {
            enter_fault(controller, "failed to start next Tower action", error);
        }
    }
}

// Init called in setup()
esp_err_t tower_sequence_controller_init(
    TowerSequenceController *controller,
    UartLink *arm_uart) {
    if (controller == NULL || arm_uart == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Controller init
    *controller = (TowerSequenceController){0};
    controller->arm_uart = arm_uart;

    // Execute the first step
    const esp_err_t error = start_tower_step(controller, 0);
    if (error != ESP_OK) {
        enter_fault(controller, "failed to start Tower sequence", error);
    }

    return error;
}

void tower_sequence_controller_update(
    TowerSequenceController *controller,
    uint32_t now_ms) {
    if (controller == NULL || !controller->running) return;

    service_arm_uart(controller);
    if (!controller->running) return;

    if ((int32_t)(now_ms - controller->action_deadline_ms) >= 0) {
        enter_fault(controller, "arm action timed out", ESP_ERR_TIMEOUT);
    }
}
