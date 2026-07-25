/* Implements the drivetrain-side Tape Following task sequence. */
#include "control/task/tape_following_sequence_controller.h"

#include <stdio.h>

// Describes one local Tape Following action and its value.
typedef struct {
    TapeFollowingAction action;
    float action_value;
} TapeFollowingSequenceStep;

// The sequence for Tape Following
static const TapeFollowingSequenceStep kTapeFollowingSequence[] = {
    {TAPE_FOLLOWING_ACTION_FOLLOW_DISTANCE, 1.0f},
    {TAPE_FOLLOWING_ACTION_ROTATE_CW_UNTIL_ALIGNED, 0.0f},
};

// The number of steps for this sequence
static const size_t kTapeFollowingSequenceLength =
    sizeof(kTapeFollowingSequence) / sizeof(kTapeFollowingSequence[0]);

// Stops the sequence and reports why it can no longer continue.
static void enter_fault(
    TapeFollowingSequenceController *controller,
    const char *reason,
    esp_err_t error) {
    controller->running = false;
    printf(
        "# Tape Following FAULT: %s (%s)\n",
        reason,
        esp_err_to_name(error));
}

// Starts one local Tape Following action.
static esp_err_t start_tape_following_step(
    TapeFollowingSequenceController *controller,
    size_t step_index) {

    // Set up the current step and its information
    const TapeFollowingSequenceStep *step =
        &kTapeFollowingSequence[step_index];

    // Start the action on the drivetrain
    const esp_err_t error = tape_following_action_controller_init(
        &controller->action_controller,
        step->action,
        step->action_value);
    if (error != ESP_OK) return error;

    controller->running = true;
    return ESP_OK;
}

// Init called in setup()
esp_err_t tape_following_sequence_controller_init(
    TapeFollowingSequenceController *controller) {
    if (controller == NULL) return ESP_ERR_INVALID_ARG;

    // Controller init
    *controller = (TapeFollowingSequenceController){0};

    // Execute the first step
    const esp_err_t error = start_tape_following_step(controller, 0);
    if (error != ESP_OK) {
        enter_fault(
            controller,
            "failed to start Tape Following sequence",
            error);
    }

    return error;
}

void tape_following_sequence_controller_update(
    TapeFollowingSequenceController *controller) {
    if (controller == NULL || !controller->running) return;

    // Execute next step if complete
    if (!tape_following_action_controller_update(
            &controller->action_controller)) {
        return;
    }

    ++controller->current_step;
    if (controller->current_step >= kTapeFollowingSequenceLength) {
        controller->running = false;
        printf("# Tape Following task sequence complete\n");
        return;
    }

    const esp_err_t error =
        start_tape_following_step(controller, controller->current_step);
    if (error != ESP_OK) {
        enter_fault(
            controller,
            "failed to start next Tape Following action",
            error);
    }
}
