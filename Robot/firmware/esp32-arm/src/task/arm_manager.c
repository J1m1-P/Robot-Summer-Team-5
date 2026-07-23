/**
 * @file arm_manager.c
 * @brief Routes arm-owned commands while keeping mechanism logic separate.
 *
 * Pickup-related action IDs share PickUpBlockAction, while BUILD_TOWER uses its
 * own implementation. This module provides the generic executor adapter and
 * never contains workflow sequencing or direct servo/stepper commands.
 */
#include "task/arm_manager.h"

#include <stddef.h>
#include <string.h>

// Groups the discrete tower-pickup steps handled by PickUpBlockAction.
static bool is_pickup_action(TaskAction action) {
    return action == TASK_ACTION_PICK_UP_BLOCK ||
           action == TASK_ACTION_POSITION_TOWER_X ||
           action == TASK_ACTION_OPEN_TOWER_CLAWS ||
           action == TASK_ACTION_TOWER_FACE_DOWN ||
           action == TASK_ACTION_LOWER_TOWER ||
           action == TASK_ACTION_CLOSE_TOWER_CLAWS ||
           action == TASK_ACTION_RAISE_TOWER ||
           action == TASK_ACTION_TOWER_FACE_FRONT;
}

// Reads the result belonging to active_action without advancing hardware.
static TaskActionResult current_result(const ArmManager *manager) {
    if (manager == NULL) {
        return (TaskActionResult){TASK_STEP_FAILED, TASK_FAILURE_PROTOCOL};
    }
    if (is_pickup_action(manager->active_action)) {
        return manager->pick_up_block.result;
    }
    if (manager->active_action == TASK_ACTION_BUILD_TOWER) {
        return manager->build_tower.result;
    }
    return (TaskActionResult){TASK_STEP_NOT_STARTED, TASK_FAILURE_NONE};
}

// Initializes each mechanism implementation and marks the manager idle.
void arm_manager_init(ArmManager *manager) {
    if (manager == NULL) return;
    memset(manager, 0, sizeof(*manager));
    manager->active_action = TASK_ACTION_COUNT;
    pick_up_block_action_init(&manager->pick_up_block);
    build_tower_action_init(&manager->build_tower);
}

// Selects and starts the implementation matching command->action.
bool arm_manager_start(ArmManager *manager, const TaskStepCommand *command,
                       uint32_t now_ms) {
    if (manager == NULL || command == NULL ||
        current_result(manager).status == TASK_STEP_RUNNING) {
        return false;
    }

    bool started = false;
    if (is_pickup_action(command->action)) {
        started = pick_up_block_action_start(
            &manager->pick_up_block, command, now_ms);
    } else if (command->action == TASK_ACTION_BUILD_TOWER) {
        started =
            build_tower_action_start(&manager->build_tower, command, now_ms);
    } else {
        return false;
    }
    if (started) manager->active_action = command->action;
    return started;
}

// Advances only the currently selected arm implementation.
void arm_manager_update(ArmManager *manager, uint32_t now_ms) {
    if (manager == NULL) return;
    if (is_pickup_action(manager->active_action)) {
        pick_up_block_action_update(&manager->pick_up_block, now_ms);
    } else if (manager->active_action == TASK_ACTION_BUILD_TOWER) {
        build_tower_action_update(&manager->build_tower, now_ms);
    }
}

// Forwards cancellation to the active implementation.
bool arm_manager_cancel(ArmManager *manager) {
    if (manager == NULL) return false;
    if (is_pickup_action(manager->active_action)) {
        return pick_up_block_action_cancel(&manager->pick_up_block);
    }
    return manager->active_action == TASK_ACTION_BUILD_TOWER
               ? build_tower_action_cancel(&manager->build_tower)
               : false;
}

// Routes supervised success injection to the active implementation.
bool arm_manager_report_succeeded(ArmManager *manager) {
    if (manager == NULL) return false;
    if (is_pickup_action(manager->active_action)) {
        return pick_up_block_action_report_succeeded(
            &manager->pick_up_block);
    }
    return manager->active_action == TASK_ACTION_BUILD_TOWER
               ? build_tower_action_report_succeeded(&manager->build_tower)
               : false;
}

// Routes a non-NONE supervised failure to the active implementation.
bool arm_manager_report_failed(ArmManager *manager, TaskFailure failure) {
    if (manager == NULL) return false;
    if (is_pickup_action(manager->active_action)) {
        return pick_up_block_action_report_failed(
            &manager->pick_up_block, failure);
    }
    return manager->active_action == TASK_ACTION_BUILD_TOWER
               ? build_tower_action_report_failed(&manager->build_tower,
                                                   failure)
               : false;
}

// Returns the current action status and optionally its failure code.
TaskStepStatus arm_manager_get_status(const ArmManager *manager,
                                      TaskFailure *failure_out) {
    const TaskActionResult result = current_result(manager);
    if (failure_out != NULL) *failure_out = result.failure;
    return result.status;
}

// Executor adapter: starts an action using the manager stored in context.
static bool executor_start(void *context, const TaskStepCommand *command,
                           uint32_t now_ms) {
    return arm_manager_start((ArmManager *)context, command, now_ms);
}

// Executor adapter: advances the manager and returns the resulting state.
static TaskActionResult executor_update(void *context, uint32_t now_ms) {
    ArmManager *manager = (ArmManager *)context;
    arm_manager_update(manager, now_ms);
    return current_result(manager);
}

// Executor adapter: cancels active work; now_ms is unused by arm mechanisms.
static void executor_cancel(void *context, uint32_t now_ms) {
    (void)now_ms;
    (void)arm_manager_cancel((ArmManager *)context);
}

// Wraps ArmManager callbacks for use by TopActionDispatcher.
TaskActionExecutor arm_manager_executor(ArmManager *manager) {
    return (TaskActionExecutor){
        .context = manager,
        .start = executor_start,
        .update = executor_update,
        .cancel = executor_cancel,
    };
}
