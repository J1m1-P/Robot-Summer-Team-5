/* Coordinates a mixed drivetrain and arm Tower task sequence. */
#pragma once

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include <robot_common/uart_link.h>

#include "control/task/tape_following_action_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TASK_DEVICE_ARM = 0,
    TASK_DEVICE_DRIVETRAIN,
} TaskDevice;

// Holds both action paths and progress for one mixed task sequence.
typedef struct {
    UartLink *arm_uart;
    TapeFollowingActionController drivetrain_action_controller;
    TaskDevice active_device;
    size_t current_step;
    uint32_t action_deadline_ms;
    bool running;
} TowerSequenceController;

// Connects the controller to its action paths and starts the first action.
esp_err_t tower_sequence_controller_init(
    TowerSequenceController *controller,
    UartLink *arm_uart);

// Processes the active action and advances or times out the sequence.
void tower_sequence_controller_update(
    TowerSequenceController *controller,
    uint32_t now_ms);

#ifdef __cplusplus
}
#endif
