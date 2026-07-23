#include "task/actions/build_tower_action.h"

#include <stddef.h>
#include <string.h>

void build_tower_action_init(BuildTowerAction *action) {
    if (action == NULL) return;
    memset(action, 0, sizeof(*action));
    action->result.status = TASK_STEP_NOT_STARTED;
}

bool build_tower_action_start(BuildTowerAction *action,
                              const TaskStepCommand *command,
                              uint32_t now_ms) {
    (void)now_ms;
    if (action == NULL || command == NULL ||
        command->action != TASK_ACTION_BUILD_TOWER ||
        action->result.status == TASK_STEP_RUNNING) {
        return false;
    }

    /* Replace this terminal placeholder with the tower mechanism's
     * nonblocking start/update/cancel implementation. */
    action->result =
        (TaskActionResult){TASK_STEP_FAILED, TASK_FAILURE_NOT_IMPLEMENTED};
    return true;
}

void build_tower_action_update(BuildTowerAction *action, uint32_t now_ms) {
    (void)action;
    (void)now_ms;
}

bool build_tower_action_cancel(BuildTowerAction *action) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING) {
        return false;
    }
    action->result =
        (TaskActionResult){TASK_STEP_CANCELLED, TASK_FAILURE_NONE};
    return true;
}

bool build_tower_action_report_succeeded(BuildTowerAction *action) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING) {
        return false;
    }
    action->result =
        (TaskActionResult){TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    return true;
}

bool build_tower_action_report_failed(BuildTowerAction *action,
                                      TaskFailure failure) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING ||
        failure == TASK_FAILURE_NONE) {
        return false;
    }
    action->result = (TaskActionResult){TASK_STEP_FAILED, failure};
    return true;
}
