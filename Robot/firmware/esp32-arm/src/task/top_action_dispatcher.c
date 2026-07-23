#include "task/top_action_dispatcher.h"

#include <stddef.h>
#include <string.h>

static bool executor_is_valid(const TaskActionExecutor *executor) {
    return executor != NULL && executor->start != NULL &&
           executor->update != NULL && executor->cancel != NULL;
}

bool top_action_dispatcher_init(TopActionDispatcher *dispatcher,
                                ArmManager *arm_manager,
                                const TaskActionExecutor *pi_scan_executor) {
    if (dispatcher == NULL || arm_manager == NULL ||
        !executor_is_valid(pi_scan_executor)) {
        return false;
    }
    memset(dispatcher, 0, sizeof(*dispatcher));
    dispatcher->arm = arm_manager_executor(arm_manager);
    dispatcher->pi_scan = *pi_scan_executor;
    return true;
}

static bool dispatcher_start(void *context, const TaskStepCommand *command,
                             uint32_t now_ms) {
    TopActionDispatcher *dispatcher = (TopActionDispatcher *)context;
    if (dispatcher == NULL || command == NULL || dispatcher->active != NULL) {
        return false;
    }
    if (command->action == TASK_ACTION_PICK_UP_BLOCK ||
        command->action == TASK_ACTION_POSITION_TOWER_X ||
        command->action == TASK_ACTION_OPEN_TOWER_CLAWS ||
        command->action == TASK_ACTION_TOWER_FACE_DOWN ||
        command->action == TASK_ACTION_LOWER_TOWER ||
        command->action == TASK_ACTION_CLOSE_TOWER_CLAWS ||
        command->action == TASK_ACTION_RAISE_TOWER ||
        command->action == TASK_ACTION_TOWER_FACE_FRONT ||
        command->action == TASK_ACTION_BUILD_TOWER) {
        dispatcher->active = &dispatcher->arm;
    } else if (command->action == TASK_ACTION_SCAN_TELETUBBIES) {
        dispatcher->active = &dispatcher->pi_scan;
    } else {
        return false;
    }
    if (!dispatcher->active->start(dispatcher->active->context, command,
                                   now_ms)) {
        dispatcher->active = NULL;
        return false;
    }
    return true;
}

static TaskActionResult dispatcher_update(void *context, uint32_t now_ms) {
    TopActionDispatcher *dispatcher = (TopActionDispatcher *)context;
    if (dispatcher == NULL || dispatcher->active == NULL) {
        return (TaskActionResult){TASK_STEP_FAILED, TASK_FAILURE_PROTOCOL};
    }
    const TaskActionResult result = dispatcher->active->update(
        dispatcher->active->context, now_ms);
    if (task_step_status_is_terminal(result.status)) dispatcher->active = NULL;
    return result;
}

static void dispatcher_cancel(void *context, uint32_t now_ms) {
    TopActionDispatcher *dispatcher = (TopActionDispatcher *)context;
    if (dispatcher == NULL || dispatcher->active == NULL) return;
    dispatcher->active->cancel(dispatcher->active->context, now_ms);
    dispatcher->active = NULL;
}

TaskActionExecutor top_action_dispatcher_executor(
    TopActionDispatcher *dispatcher) {
    return (TaskActionExecutor){
        .context = dispatcher,
        .start = dispatcher_start,
        .update = dispatcher_update,
        .cancel = dispatcher_cancel,
    };
}
