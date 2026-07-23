/**
 * @file task.c
 * @brief Implements transport-independent validation for shared task values.
 *
 * These checks protect coordinator, dispatcher, and wire-protocol boundaries
 * from malformed requests or sentinel enum values. No hardware or mutable
 * runtime behavior belongs in this module.
 */
#include <robot_common/task/task.h>

#include <math.h>
#include <stddef.h>

// Validates only the parameters used by the selected task type.
bool task_request_is_valid(const TaskRequest *request) {
    if (request == NULL) return false;
    for (uint8_t step = 0; step < TASK_MAX_STEPS; ++step) {
        if (!isfinite(request->step_parameters[step].amount) ||
            !isfinite(request->step_parameters[step].speed)) {
            return false;
        }
    }

    if (request->type < TASK_TYPE_TAPE_FOLLOWING ||
        request->type >= TASK_TYPE_COUNT) {
        return false;
    }

    if (request->type != TASK_TYPE_TAPE_FOLLOWING) return true;

    const TaskStepParameters *params = &request->step_parameters[0];
    return (request->step_parameter_override_mask & UINT16_C(1)) != 0U &&
           params->amount != 0.0f && params->speed > 0.0f;
}

// Rejects sentinel or out-of-range actions at module boundaries.
bool task_action_is_valid(TaskAction action) {
    return action >= TASK_ACTION_FOLLOW_TAPE && action < TASK_ACTION_COUNT &&
           action != TASK_ACTION_RESERVED;
}

// Identifies results that permanently close an individual workflow step.
bool task_step_status_is_terminal(TaskStepStatus status) {
    return status == TASK_STEP_SUCCEEDED || status == TASK_STEP_CANCELLED ||
           status == TASK_STEP_FAILED;
}
