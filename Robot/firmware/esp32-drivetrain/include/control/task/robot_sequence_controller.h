/* Coordinates one ordered sequence of drivetrain and arm actions. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include <robot_common/uart_link.h>

#include "control/task/tape_following_action_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    UartLink *arm_uart;
    TapeFollowingActionController tape_action_controller;
    size_t current_step;
    uint32_t step_deadline_ms;
    bool running;
} RobotSequenceController;

// Connects the action paths and starts the first robot step.
esp_err_t robot_sequence_controller_init(
    RobotSequenceController *controller,
    UartLink *arm_uart);

// Updates only the current step and advances when it completes.
void robot_sequence_controller_update(
    RobotSequenceController *controller,
    uint32_t now_ms);

#ifdef __cplusplus
}
#endif
