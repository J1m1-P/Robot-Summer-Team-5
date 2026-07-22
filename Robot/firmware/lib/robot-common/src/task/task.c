/** @file task.c
 *  @brief Validates task requests and owns immutable workflow definitions.
 */
#include <robot_common/task/task.h>

#include <math.h>
#include <stddef.h>
#include <string.h>

static const TaskStepDefinition TAPE_WORKFLOW[] = {
    {TASK_ACTION_FOLLOW_TAPE, TASK_OWNER_DRIVETRAIN},
};

static const TaskStepDefinition PICKING_WORKFLOW[] = {
    {TASK_ACTION_ALIGN_TO_PIECES, TASK_OWNER_DRIVETRAIN},
    {TASK_ACTION_PICK_UP_BLOCK, TASK_OWNER_ARM},
    {TASK_ACTION_ALIGN_TO_TAPE, TASK_OWNER_DRIVETRAIN},
};

static const TaskStepDefinition BUILDING_WORKFLOW[] = {
    {TASK_ACTION_ALIGN_TO_BASE, TASK_OWNER_DRIVETRAIN},
    {TASK_ACTION_BUILD_TOWER, TASK_OWNER_ARM},
    {TASK_ACTION_ALIGN_TO_TAPE, TASK_OWNER_DRIVETRAIN},
};

static const TaskStepDefinition ROUTINE_WORKFLOW[] = {
    {TASK_ACTION_FOLLOW_TAPE, TASK_OWNER_DRIVETRAIN},
    {TASK_ACTION_ALIGN_TO_PIECES, TASK_OWNER_DRIVETRAIN},
    {TASK_ACTION_PICK_UP_BLOCK, TASK_OWNER_ARM},
    {TASK_ACTION_ALIGN_TO_TAPE, TASK_OWNER_DRIVETRAIN},
    {TASK_ACTION_FOLLOW_TAPE, TASK_OWNER_DRIVETRAIN},
    {TASK_ACTION_ALIGN_TO_BASE, TASK_OWNER_DRIVETRAIN},
    {TASK_ACTION_BUILD_TOWER, TASK_OWNER_ARM},
    {TASK_ACTION_ALIGN_TO_TAPE, TASK_OWNER_DRIVETRAIN},
};

typedef struct {
    const TaskStepDefinition *steps;
    uint8_t count;
} WorkflowDefinition;

static bool tape_params_are_valid(const TapeFollowingTaskParams *params) {
    return params != NULL &&
           (params->direction == TAPE_DIRECTION_FORWARD ||
            params->direction == TAPE_DIRECTION_REVERSE) &&
           isfinite(params->speed_mps) && params->speed_mps > 0.0f &&
           isfinite(params->distance_m) && params->distance_m > 0.0f;
}

static bool get_workflow(TaskType type, WorkflowDefinition *workflow_out) {
    if (workflow_out == NULL) return false;

    switch (type) {
        case TASK_TYPE_TAPE_FOLLOWING:
            workflow_out->steps = TAPE_WORKFLOW;
            workflow_out->count = (uint8_t)(sizeof(TAPE_WORKFLOW) /
                                             sizeof(TAPE_WORKFLOW[0]));
            return true;
        case TASK_TYPE_TOWER_PICKING:
            workflow_out->steps = PICKING_WORKFLOW;
            workflow_out->count = (uint8_t)(sizeof(PICKING_WORKFLOW) /
                                             sizeof(PICKING_WORKFLOW[0]));
            return true;
        case TASK_TYPE_TOWER_BUILDING:
            workflow_out->steps = BUILDING_WORKFLOW;
            workflow_out->count = (uint8_t)(sizeof(BUILDING_WORKFLOW) /
                                             sizeof(BUILDING_WORKFLOW[0]));
            return true;
        case TASK_TYPE_TOWER_ROUTINE:
            workflow_out->steps = ROUTINE_WORKFLOW;
            workflow_out->count = (uint8_t)(sizeof(ROUTINE_WORKFLOW) /
                                             sizeof(ROUTINE_WORKFLOW[0]));
            return true;
        default:
            return false;
    }
}

bool task_request_is_valid(const TaskRequest *request) {
    if (request == NULL) return false;

    switch (request->type) {
        case TASK_TYPE_TAPE_FOLLOWING:
            return tape_params_are_valid(&request->params.tape_following);
        case TASK_TYPE_TOWER_PICKING:
        case TASK_TYPE_TOWER_BUILDING:
            return true;
        case TASK_TYPE_TOWER_ROUTINE:
            return tape_params_are_valid(
                       &request->params.tower_routine.tape_to_pieces) &&
                   tape_params_are_valid(
                       &request->params.tower_routine.tape_to_base);
        default:
            return false;
    }
}

bool task_get_step_count(TaskType type, uint8_t *count_out) {
    WorkflowDefinition workflow = {0};
    if (count_out == NULL || !get_workflow(type, &workflow)) return false;
    *count_out = workflow.count;
    return true;
}

bool task_get_step_definition(TaskType type, uint8_t step,
                              TaskStepDefinition *definition_out) {
    WorkflowDefinition workflow = {0};
    if (definition_out == NULL || !get_workflow(type, &workflow) ||
        step >= workflow.count) {
        return false;
    }
    *definition_out = workflow.steps[step];
    return true;
}

bool task_build_step_command(const TaskRuntime *runtime,
                             TaskStepCommand *command_out) {
    if (runtime == NULL || command_out == NULL ||
        runtime->status != TASK_STATUS_RUNNING) {
        return false;
    }

    TaskStepDefinition definition = {0};
    if (!task_get_step_definition(runtime->request.type,
                                  runtime->current_step, &definition)) {
        return false;
    }

    memset(command_out, 0, sizeof(*command_out));
    command_out->execution_id = runtime->execution_id;
    command_out->step = runtime->current_step;
    command_out->action = definition.action;

    if (definition.action == TASK_ACTION_FOLLOW_TAPE) {
        if (runtime->request.type == TASK_TYPE_TAPE_FOLLOWING) {
            command_out->params.tape_following =
                runtime->request.params.tape_following;
        } else if (runtime->request.type == TASK_TYPE_TOWER_ROUTINE) {
            command_out->params.tape_following =
                runtime->current_step == 0U
                    ? runtime->request.params.tower_routine.tape_to_pieces
                    : runtime->request.params.tower_routine.tape_to_base;
        } else {
            return false;
        }
    }
    return true;
}

bool task_owner_is_valid(TaskOwner owner) {
    return owner >= TASK_OWNER_DRIVETRAIN && owner < TASK_OWNER_COUNT;
}

bool task_action_is_valid(TaskAction action) {
    return action >= TASK_ACTION_FOLLOW_TAPE && action < TASK_ACTION_COUNT;
}

bool task_step_status_is_terminal(TaskStepStatus status) {
    return status == TASK_STEP_SUCCEEDED || status == TASK_STEP_CANCELLED ||
           status == TASK_STEP_FAILED;
}
