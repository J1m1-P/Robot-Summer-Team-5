/* Coordinates the drivetrain-side Tape Following task sequence. */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#include "control/task/tape_following_action_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

// Holds the local action controller and progress for one Tape Following sequence.
typedef struct {
    TapeFollowingActionController action_controller;
    size_t current_step;
    bool running;
} TapeFollowingSequenceController;

// Connects the sequence to its action controller and starts the first action.
esp_err_t tape_following_sequence_controller_init(
    TapeFollowingSequenceController *controller);

// Processes the local action and advances when the active action completes.
void tape_following_sequence_controller_update(
    TapeFollowingSequenceController *controller);

#ifdef __cplusplus
}
#endif
