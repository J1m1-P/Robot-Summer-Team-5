#include "task/actions/pick_up_block_action.h"

#include <stddef.h>
#include <string.h>

void pick_up_block_action_init(PickUpBlockAction *action) {
    if (action == NULL) return;
    memset(action, 0, sizeof(*action));
    action->result.status = TASK_STEP_NOT_STARTED;
}

bool pick_up_block_action_start(PickUpBlockAction *action,
                                const TaskStepCommand *command,
                                uint32_t now_ms) {
    (void)now_ms;
    if (action == NULL || command == NULL ||
        command->action != TASK_ACTION_PICK_UP_BLOCK ||
        action->result.status == TASK_STEP_RUNNING) {
        return false;
    }

    /* Replace this terminal placeholder with the pickup mechanism's
     * nonblocking start/update/cancel implementation. */
    action->result =
        (TaskActionResult){TASK_STEP_FAILED, TASK_FAILURE_NOT_IMPLEMENTED};
    return true;
}

void pick_up_block_action_update(PickUpBlockAction *action, uint32_t now_ms) {
    (void)action;
    (void)now_ms;
}

bool pick_up_block_action_cancel(PickUpBlockAction *action) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING) {
        return false;
    }
    action->result =
        (TaskActionResult){TASK_STEP_CANCELLED, TASK_FAILURE_NONE};
    return true;
}

bool pick_up_block_action_report_succeeded(PickUpBlockAction *action) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING) {
        return false;
    }
    action->result =
        (TaskActionResult){TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    return true;
}

bool pick_up_block_action_report_failed(PickUpBlockAction *action,
                                        TaskFailure failure) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING ||
        failure == TASK_FAILURE_NONE) {
        return false;
    }
    action->result = (TaskActionResult){TASK_STEP_FAILED, failure};
    return true;
}
