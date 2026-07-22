#include "task/arm_manager.h"

#include <stddef.h>
#include <string.h>

// Keeps drivetrain-owned actions from entering the arm mechanism layer.
static bool action_is_arm_owned(TaskAction action) {
    return action == TASK_ACTION_PICK_UP_BLOCK ||
           action == TASK_ACTION_BUILD_TOWER;
}

// Resets the single-action arm execution slot to its inactive state.
void arm_manager_init(ArmManager *manager) {
    if (manager == NULL) return;
    memset(manager, 0, sizeof(*manager));
    manager->status = TASK_STEP_NOT_STARTED;
}

// Accepts one arm-owned command when no other arm action is active.
bool arm_manager_start(ArmManager *manager, const TaskStepCommand *command) {
    if (manager == NULL || command == NULL ||
        manager->status == TASK_STEP_RUNNING ||
        !action_is_arm_owned(command->action)) {
        return false;
    }
    manager->status = TASK_STEP_RUNNING;
    manager->failure = TASK_FAILURE_NONE;
    return true;
}

// Stops the active logical action; physical mechanism shutdown belongs in this hook too.
bool arm_manager_cancel(ArmManager *manager) {
    if (manager == NULL || manager->status != TASK_STEP_RUNNING) return false;
    /* Future mechanism drivers must stop outputs before this returns. */
    manager->status = TASK_STEP_CANCELLED;
    manager->failure = TASK_FAILURE_NONE;
    return true;
}

// Publishes successful physical completion to the task server.
bool arm_manager_report_succeeded(ArmManager *manager) {
    if (manager == NULL || manager->status != TASK_STEP_RUNNING) {
        return false;
    }
    manager->status = TASK_STEP_SUCCEEDED;
    manager->failure = TASK_FAILURE_NONE;
    return true;
}

// Publishes a nonzero physical failure and closes the active action.
bool arm_manager_report_failed(ArmManager *manager, TaskFailure failure) {
    if (manager == NULL || manager->status != TASK_STEP_RUNNING ||
        failure == TASK_FAILURE_NONE) {
        return false;
    }
    manager->status = TASK_STEP_FAILED;
    manager->failure = failure;
    return true;
}

// Returns the current action result without transferring ownership of manager state.
TaskStepStatus arm_manager_get_status(const ArmManager *manager,
                                      TaskFailure *failure_out) {
    if (manager == NULL) {
        if (failure_out != NULL) *failure_out = TASK_FAILURE_PROTOCOL;
        return TASK_STEP_FAILED;
    }
    if (failure_out != NULL) *failure_out = manager->failure;
    return manager->status;
}
