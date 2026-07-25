/* Coordinates the drivetrain-side Tower demo sequence over the arm UART. */
#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include <robot_common/uart_link.h>

#ifdef __cplusplus
extern "C" {
#endif

// Holds the UART connection and progress for one Tower action sequence.
typedef struct {
    UartLink *arm_uart;
    size_t current_step;
    uint32_t action_deadline_ms;
    bool running;
} TowerSequenceController;

// Connects the controller to the arm UART and starts the first action.
esp_err_t tower_sequence_controller_init(
    TowerSequenceController *controller,
    UartLink *arm_uart);

// Processes arm status packets and advances or times out the active action.
void tower_sequence_controller_update(
    TowerSequenceController *controller,
    uint32_t now_ms);

#ifdef __cplusplus
}
#endif
