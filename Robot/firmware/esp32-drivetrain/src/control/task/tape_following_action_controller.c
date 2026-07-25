/* Implements the placeholder actions used by the Tape Following task. */
#include "control/task/tape_following_action_controller.h"

#include <stddef.h>
#include <stdio.h>

esp_err_t tape_following_action_controller_init(
    TapeFollowingActionController *controller,
    TapeFollowingAction action,
    float action_value) {
    if (controller == NULL ||
        action > TAPE_FOLLOWING_ACTION_STEP_3) {
        return ESP_ERR_INVALID_ARG;
    }

    // Controller init
    *controller = (TapeFollowingActionController){0};
    controller->action = action;
    controller->action_value = action_value;
    return ESP_OK;
}

bool tape_following_action_controller_update(
    TapeFollowingActionController *controller) {
    if (controller == NULL) return false;

    // Decide what to do for this action
    switch (controller->action) {
        case TAPE_FOLLOWING_ACTION_FOLLOW_DISTANCE:
            printf(
                "# PLACEHOLDER: Tape following for %.1f m\n",
                controller->action_value);
            break;

        case TAPE_FOLLOWING_ACTION_ROTATE_CW_UNTIL_ALIGNED:
            printf("# PLACEHOLDER: Rotate CW until aligned with tape again\n");
            break;

        case TAPE_FOLLOWING_ACTION_STEP_3:
            printf("# PLACEHOLDER: Tape Following step 3\n");
            break;

        default:
            return false;
    }

    // Replace this immediate completion when the motion is implemented
    return true;
}
