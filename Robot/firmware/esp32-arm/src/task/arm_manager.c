#include "task/arm_manager.h"

#include <stddef.h>
#include <string.h>

static bool action_is_arm_owned(TaskAction action) {
    return action == TASK_ACTION_PICK_UP_BLOCK ||
           action == TASK_ACTION_BUILD_TOWER;
}

void arm_manager_init(ArmManager *manager) {
    if (manager == NULL) return;
    memset(manager, 0, sizeof(*manager));
    manager->status = TASK_STEP_NOT_STARTED;
}

bool arm_manager_start(ArmManager *manager, const TaskStepCommand *command) {
    if (manager == NULL || command == NULL || manager->active ||
        !action_is_arm_owned(command->action)) {
        return false;
    }
    manager->command = *command;
    manager->status = TASK_STEP_RUNNING;
    manager->failure = TASK_FAILURE_NONE;
    manager->active = true;
    return true;
}

void arm_manager_update(ArmManager *manager) {
    (void)manager;
    /* The mechanism controller reports completion through report_*(). */
}

bool arm_manager_cancel(ArmManager *manager) {
    if (manager == NULL || !manager->active) return false;
    /* Future mechanism drivers must stop outputs before this returns. */
    manager->status = TASK_STEP_CANCELLED;
    manager->failure = TASK_FAILURE_NONE;
    manager->active = false;
    return true;
}

bool arm_manager_report_succeeded(ArmManager *manager) {
    if (manager == NULL || !manager->active ||
        manager->status != TASK_STEP_RUNNING) {
        return false;
    }
    manager->status = TASK_STEP_SUCCEEDED;
    manager->failure = TASK_FAILURE_NONE;
    manager->active = false;
    return true;
}

bool arm_manager_report_failed(ArmManager *manager, TaskFailure failure) {
    if (manager == NULL || !manager->active || failure == TASK_FAILURE_NONE) {
        return false;
    }
    manager->status = TASK_STEP_FAILED;
    manager->failure = failure;
    manager->active = false;
    return true;
}

TaskStepStatus arm_manager_get_status(const ArmManager *manager,
                                      TaskFailure *failure_out) {
    if (manager == NULL) {
        if (failure_out != NULL) *failure_out = TASK_FAILURE_PROTOCOL;
        return TASK_STEP_FAILED;
    }
    if (failure_out != NULL) *failure_out = manager->failure;
    return manager->status;
}
