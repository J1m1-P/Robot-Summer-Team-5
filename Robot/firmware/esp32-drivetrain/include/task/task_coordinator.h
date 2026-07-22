/** @file task_coordinator.h
 *  @brief Sole owner of task sequencing and authoritative runtime state.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "task/task_action_executor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t step_timeout_ms;
} TaskCoordinatorConfig;

typedef struct {
    TaskCoordinatorConfig config;
    TaskActionExecutor executors[TASK_OWNER_COUNT];
    TaskRuntime runtime;
    uint32_t step_started_ms;
} TaskCoordinator;

bool task_coordinator_init(TaskCoordinator *coordinator,
                           const TaskCoordinatorConfig *config,
                           const TaskActionExecutor *drivetrain_executor,
                           const TaskActionExecutor *arm_executor);
bool task_coordinator_start(TaskCoordinator *coordinator,
                            const TaskRequest *request,
                            uint32_t execution_id,
                            uint32_t now_ms);
void task_coordinator_update(TaskCoordinator *coordinator, uint32_t now_ms);
bool task_coordinator_cancel(TaskCoordinator *coordinator, uint32_t now_ms);
bool task_coordinator_fail(TaskCoordinator *coordinator,
                           TaskFailure failure,
                           uint32_t now_ms);
const TaskRuntime *task_coordinator_get_runtime(
    const TaskCoordinator *coordinator);
bool task_coordinator_get_current_owner(const TaskCoordinator *coordinator,
                                        TaskOwner *owner_out);

#ifdef __cplusplus
}
#endif
