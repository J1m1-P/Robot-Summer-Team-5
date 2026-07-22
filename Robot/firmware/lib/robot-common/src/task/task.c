/** @file task.c
 *  @brief Validates task requests and provides task step metadata.
 */
#include <robot_common/task/task.h>

#include <stddef.h>

/**
 * @brief Validates a task request and its type-specific parameters.
 * @param request Request to validate.
 * @return true when the request can be started.
 */
bool task_request_is_valid(const TaskRequest *request) {
    if (request == NULL) return false;

    switch (request->type) {
        case TASK_TYPE_TOWER_PICKING:
        case TASK_TYPE_TOWER_BUILDING:
            return true;

        case TASK_TYPE_TAPE_FOLLOWING:
            return (request->params.tape_following.direction ==
                        TAPE_DIRECTION_FORWARD ||
                    request->params.tape_following.direction ==
                        TAPE_DIRECTION_REVERSE) &&
                   request->params.tape_following.speed_mps > 0.0f &&
                   request->params.tape_following.distance_m > 0.0f;

        default:
            return false;
    }
}

/**
 * @brief Gets the owner of a step for a supported task type.
 * @param type Task workflow containing the step.
 * @param step Workflow step index.
 * @param owner_out Receives the responsible subsystem.
 * @return true when the type and step are valid.
 */
bool task_get_step_owner(TaskType type, uint16_t step,
                         TaskOwner *owner_out) {
    if (owner_out == NULL) return false;

    switch (type) {
        case TASK_TYPE_TOWER_PICKING:
            switch (step) {
                case TOWER_PICKING_STEP_ALIGN_TO_PIECES:
                case TOWER_PICKING_STEP_ALIGN_TO_TAPE:
                    *owner_out = TASK_OWNER_DRIVETRAIN;
                    return true;
                case TOWER_PICKING_STEP_PICK_UP_BLOCK:
                    *owner_out = TASK_OWNER_ARM;
                    return true;
                default:
                    return false;
            }

        case TASK_TYPE_TOWER_BUILDING:
            switch (step) {
                case TOWER_BUILDING_STEP_ALIGN_TO_BASE:
                case TOWER_BUILDING_STEP_ALIGN_TO_TAPE:
                    *owner_out = TASK_OWNER_DRIVETRAIN;
                    return true;
                case TOWER_BUILDING_STEP_BUILD_TOWER:
                    *owner_out = TASK_OWNER_ARM;
                    return true;
                default:
                    return false;
            }

        case TASK_TYPE_TAPE_FOLLOWING:
            if (step != TAPE_FOLLOWING_STEP_FOLLOW_TAPE) return false;
            *owner_out = TASK_OWNER_DRIVETRAIN;
            return true;

        default:
            return false;
    }
}

/**
 * @brief Gets the number of steps for a supported task type.
 * @param type Task workflow to inspect.
 * @param count_out Receives the workflow step count.
 * @return true when the task type is supported.
 */
bool task_get_step_count(TaskType type, uint16_t *count_out) {
    if (count_out == NULL) return false;

    switch (type) {
        case TASK_TYPE_TOWER_PICKING:
            *count_out = TOWER_PICKING_STEP_COUNT;
            return true;
        case TASK_TYPE_TOWER_BUILDING:
            *count_out = TOWER_BUILDING_STEP_COUNT;
            return true;
        case TASK_TYPE_TAPE_FOLLOWING:
            *count_out = TAPE_FOLLOWING_STEP_COUNT;
            return true;
        default:
            return false;
    }
}
