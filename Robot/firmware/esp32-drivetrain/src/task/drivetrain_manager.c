#include "task/drivetrain_manager.h"

#include <stddef.h>
#include <string.h>

// Keeps physical drivetrain actions from being accepted by the arm-side executor.
static bool action_is_drivetrain_owned(TaskAction action) {
    return action == TASK_ACTION_FOLLOW_TAPE ||
           action == TASK_ACTION_ALIGN_TO_PIECES ||
           action == TASK_ACTION_ALIGN_TO_TAPE ||
           action == TASK_ACTION_ALIGN_TO_BASE;
}

// Accepts one drivetrain-owned action when no action is already running.
static bool manager_start(void *context, const TaskStepCommand *command,
                          uint32_t now_ms) {
    (void)now_ms;
    DrivetrainManager *manager = (DrivetrainManager *)context;
    if (manager == NULL || command == NULL ||
        manager->result.status == TASK_STEP_RUNNING ||
        !action_is_drivetrain_owned(command->action)) {
        return false;
    }
    manager->result.status = TASK_STEP_RUNNING;
    manager->result.failure = TASK_FAILURE_NONE;
    return true;
}

// Returns the latest result reported by the future nonblocking behavior.
static TaskActionResult manager_update(void *context, uint32_t now_ms) {
    (void)now_ms;
    DrivetrainManager *manager = (DrivetrainManager *)context;
    if (manager == NULL || manager->result.status == TASK_STEP_NOT_STARTED) {
        const TaskActionResult invalid = {
            TASK_STEP_FAILED, TASK_FAILURE_PROTOCOL};
        return invalid;
    }
    return manager->result;
}

// Brakes initialized hardware before publishing a cancelled result.
static void manager_cancel(void *context, uint32_t now_ms) {
    (void)now_ms;
    DrivetrainManager *manager = (DrivetrainManager *)context;
    if (manager == NULL) return;
    if (manager->drivetrain != NULL &&
        manager->drivetrain->status.initialized) {
        (void)drivetrain_brake(manager->drivetrain);
    }
    manager->result.status = TASK_STEP_CANCELLED;
    manager->result.failure = TASK_FAILURE_NONE;
}

// Resets manager state and binds the drivetrain used for fail-safe braking.
void drivetrain_manager_init(DrivetrainManager *manager,
                             Drivetrain *drivetrain) {
    if (manager == NULL) return;
    memset(manager, 0, sizeof(*manager));
    manager->drivetrain = drivetrain;
    manager->result.status = TASK_STEP_NOT_STARTED;
}

// Adapts the concrete manager to the coordinator's generic executor callbacks.
TaskActionExecutor drivetrain_manager_executor(DrivetrainManager *manager) {
    const TaskActionExecutor executor = {
        .context = manager,
        .start = manager_start,
        .update = manager_update,
        .cancel = manager_cancel,
    };
    return executor;
}

// Lets nonblocking drivetrain behavior report successful physical completion.
bool drivetrain_manager_report_succeeded(DrivetrainManager *manager) {
    if (manager == NULL || manager->result.status != TASK_STEP_RUNNING) {
        return false;
    }
    manager->result.status = TASK_STEP_SUCCEEDED;
    return true;
}

// Records a physical failure and brakes the drivetrain immediately.
bool drivetrain_manager_report_failed(DrivetrainManager *manager,
                                      TaskFailure failure) {
    if (manager == NULL || manager->result.status != TASK_STEP_RUNNING ||
        failure == TASK_FAILURE_NONE) {
        return false;
    }
    manager->result.status = TASK_STEP_FAILED;
    manager->result.failure = failure;
    if (manager->drivetrain != NULL &&
        manager->drivetrain->status.initialized) {
        (void)drivetrain_brake(manager->drivetrain);
    }
    return true;
}
