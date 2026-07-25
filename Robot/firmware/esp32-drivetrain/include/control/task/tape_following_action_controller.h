/* Declares the drivetrain-side actions used by the Tape Following task. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TAPE_FOLLOWING_ACTION_FOLLOW_DISTANCE = 0,
    TAPE_FOLLOWING_ACTION_ROTATE_CW_UNTIL_ALIGNED,
    TAPE_FOLLOWING_ACTION_STEP_3,
} TapeFollowingAction;

typedef struct {
    TapeFollowingAction action;
    float action_value;
} TapeFollowingActionController;

// Prepares one action. The motion implementations are intentionally placeholders.
esp_err_t tape_following_action_controller_init(
    TapeFollowingActionController *controller,
    TapeFollowingAction action,
    float action_value);

// Updates the active action and returns true when it is complete.
bool tape_following_action_controller_update(
    TapeFollowingActionController *controller);

#ifdef __cplusplus
}
#endif
