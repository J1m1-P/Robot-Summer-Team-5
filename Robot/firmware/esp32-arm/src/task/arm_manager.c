/** @file arm_manager.c
 *  @brief Dispatches arm-owned actions without implementing their mechanics.
 */
#include "task/arm_manager.h"

#include <stddef.h>
#include <string.h>

static TaskActionResult current_result(const ArmManager *manager) {
    if (manager == NULL) {
        return (TaskActionResult){TASK_STEP_FAILED, TASK_FAILURE_PROTOCOL};
    }
    switch (manager->active_action) {
        case TASK_ACTION_PICK_UP_BLOCK:
            return manager->pick_up_block.result;
        case TASK_ACTION_BUILD_TOWER:
            return manager->build_tower.result;
        default:
            return (TaskActionResult){TASK_STEP_NOT_STARTED,
                                      TASK_FAILURE_NONE};
    }
}

void arm_manager_init(ArmManager *manager) {
    if (manager == NULL) return;
    memset(manager, 0, sizeof(*manager));
    manager->active_action = TASK_ACTION_COUNT;
    pick_up_block_action_init(&manager->pick_up_block);
    build_tower_action_init(&manager->build_tower);
}

bool arm_manager_start(ArmManager *manager, const TaskStepCommand *command,
                       uint32_t now_ms) {
    if (manager == NULL || command == NULL ||
        current_result(manager).status == TASK_STEP_RUNNING) {
        return false;
    }

    bool started = false;
    switch (command->action) {
        case TASK_ACTION_PICK_UP_BLOCK:
            started = pick_up_block_action_start(
                &manager->pick_up_block, command, now_ms);
            break;
        case TASK_ACTION_BUILD_TOWER:
            started =
                build_tower_action_start(&manager->build_tower, command, now_ms);
            break;
        default:
            return false;
    }
    if (started) manager->active_action = command->action;
    return started;
}

void arm_manager_update(ArmManager *manager, uint32_t now_ms) {
    if (manager == NULL) return;
    switch (manager->active_action) {
        case TASK_ACTION_PICK_UP_BLOCK:
            pick_up_block_action_update(&manager->pick_up_block, now_ms);
            break;
        case TASK_ACTION_BUILD_TOWER:
            build_tower_action_update(&manager->build_tower, now_ms);
            break;
        default:
            break;
    }
}

bool arm_manager_cancel(ArmManager *manager) {
    if (manager == NULL) return false;
    switch (manager->active_action) {
        case TASK_ACTION_PICK_UP_BLOCK:
            return pick_up_block_action_cancel(&manager->pick_up_block);
        case TASK_ACTION_BUILD_TOWER:
            return build_tower_action_cancel(&manager->build_tower);
        default:
            return false;
    }
}

bool arm_manager_report_succeeded(ArmManager *manager) {
    if (manager == NULL) return false;
    switch (manager->active_action) {
        case TASK_ACTION_PICK_UP_BLOCK:
            return pick_up_block_action_report_succeeded(
                &manager->pick_up_block);
        case TASK_ACTION_BUILD_TOWER:
            return build_tower_action_report_succeeded(&manager->build_tower);
        default:
            return false;
    }
}

bool arm_manager_report_failed(ArmManager *manager, TaskFailure failure) {
    if (manager == NULL) return false;
    switch (manager->active_action) {
        case TASK_ACTION_PICK_UP_BLOCK:
            return pick_up_block_action_report_failed(
                &manager->pick_up_block, failure);
        case TASK_ACTION_BUILD_TOWER:
            return build_tower_action_report_failed(&manager->build_tower,
                                                    failure);
        default:
            return false;
    }
}

TaskStepStatus arm_manager_get_status(const ArmManager *manager,
                                      TaskFailure *failure_out) {
    const TaskActionResult result = current_result(manager);
    if (failure_out != NULL) *failure_out = result.failure;
    return result.status;
}

static bool executor_start(void *context, const TaskStepCommand *command,
                           uint32_t now_ms) {
    return arm_manager_start((ArmManager *)context, command, now_ms);
}

static TaskActionResult executor_update(void *context, uint32_t now_ms) {
    ArmManager *manager = (ArmManager *)context;
    arm_manager_update(manager, now_ms);
    return current_result(manager);
}

static void executor_cancel(void *context, uint32_t now_ms) {
    (void)now_ms;
    (void)arm_manager_cancel((ArmManager *)context);
}

TaskActionExecutor arm_manager_executor(ArmManager *manager) {
    return (TaskActionExecutor){
        .context = manager,
        .start = executor_start,
        .update = executor_update,
        .cancel = executor_cancel,
    };
}
