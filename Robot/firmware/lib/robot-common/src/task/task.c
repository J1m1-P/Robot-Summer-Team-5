/** @file task.c
 *  @brief Validates task requests and shared task enum values.
 */
#include <robot_common/task/task.h>

#include <math.h>
#include <stddef.h>

// Validates the physical units and direction required by a tape-following action.
static bool tape_params_are_valid(const TapeFollowingTaskParams *params) {
    return params != NULL &&
           (params->direction == TAPE_DIRECTION_FORWARD ||
            params->direction == TAPE_DIRECTION_REVERSE) &&
           isfinite(params->speed_mps) && params->speed_mps > 0.0f &&
           isfinite(params->distance_m) && params->distance_m > 0.0f;
}

// Validates only the parameters used by the selected task type.
bool task_request_is_valid(const TaskRequest *request) {
    if (request == NULL) return false;

    switch (request->type) {
        case TASK_TYPE_TAPE_FOLLOWING:
            return tape_params_are_valid(&request->params.tape_following);
        case TASK_TYPE_TOWER_PICKING:
        case TASK_TYPE_TOWER_BUILDING:
        case TASK_TYPE_TELETUBBY_SCAN:
            return true;
        default:
            return false;
    }
}

// Rejects sentinel or out-of-range actions at module boundaries.
bool task_action_is_valid(TaskAction action) {
    return action == TASK_ACTION_FOLLOW_TAPE ||
           action == TASK_ACTION_ALIGN_TO_PIECES ||
           action == TASK_ACTION_PICK_UP_BLOCK ||
           action == TASK_ACTION_ALIGN_TO_TAPE ||
           action == TASK_ACTION_BUILD_TOWER ||
           action == TASK_ACTION_SCAN_TELETUBBIES;
}

// Identifies results that permanently close an individual workflow step.
bool task_step_status_is_terminal(TaskStepStatus status) {
    return status == TASK_STEP_SUCCEEDED || status == TASK_STEP_CANCELLED ||
           status == TASK_STEP_FAILED;
}
