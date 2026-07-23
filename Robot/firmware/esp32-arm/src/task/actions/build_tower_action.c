/**
 * @file build_tower_action.c
 * @brief Provides the routed placeholder for future tower-building mechanics.
 *
 * The action already conforms to the manager's nonblocking lifecycle API, but
 * start currently produces NOT_IMPLEMENTED. Replace that terminal placeholder
 * with mechanism-specific start/update/cancel behavior when hardware is ready.
 */
#include "task/actions/build_tower_action.h"

#include <stddef.h>
#include <string.h>

// Clears placeholder state and leaves the action idle.
void build_tower_action_init(BuildTowerAction *action) {
    if (action == NULL) return;
    memset(action, 0, sizeof(*action));
    action->result.status = TASK_STEP_NOT_STARTED;
}

// Validates BUILD_TOWER and currently records a terminal NOT_IMPLEMENTED result.
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

// Reserved poll point for future nonblocking tower-building work.
void build_tower_action_update(BuildTowerAction *action, uint32_t now_ms) {
    (void)action;
    (void)now_ms;
}

// Cancels future running work; the current placeholder never remains running.
bool build_tower_action_cancel(BuildTowerAction *action) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING) {
        return false;
    }
    action->result =
        (TaskActionResult){TASK_STEP_CANCELLED, TASK_FAILURE_NONE};
    return true;
}

// Allows a supervised test harness to force successful completion.
bool build_tower_action_report_succeeded(BuildTowerAction *action) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING) {
        return false;
    }
    action->result =
        (TaskActionResult){TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    return true;
}

// Allows a supervised test harness to inject a mechanism failure.
bool build_tower_action_report_failed(BuildTowerAction *action,
                                      TaskFailure failure) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING ||
        failure == TASK_FAILURE_NONE) {
        return false;
    }
    action->result = (TaskActionResult){TASK_STEP_FAILED, failure};
    return true;
}
