/** @file task_coordinator.h
 *  @brief Sole owner of task sequencing and authoritative runtime state.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/task/task_action_executor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t step_timeout_ms;
} TaskCoordinatorConfig;

typedef bool (*TaskEnterSafeState)(void *context);

typedef struct {
    void *context;
    TaskEnterSafeState enter;
} TaskSafeStateHandler;

typedef struct {
    TaskCoordinatorConfig config;
    TaskActionExecutor executors[TASK_OWNER_COUNT];
    TaskSafeStateHandler safe_state;
    TaskRuntime runtime;
    uint32_t step_started_ms;
} TaskCoordinator;

bool task_coordinator_init(TaskCoordinator *coordinator,
                           const TaskCoordinatorConfig *config,
                           const TaskActionExecutor *drivetrain_executor,
                           const TaskActionExecutor *top_executor,
                           const TaskSafeStateHandler *safe_state);
bool task_coordinator_start(TaskCoordinator *coordinator,
                            const TaskRequest *request,
                            uint32_t execution_id);
void task_coordinator_update(TaskCoordinator *coordinator, uint32_t now_ms);
bool task_coordinator_cancel(TaskCoordinator *coordinator, uint32_t now_ms);
bool task_coordinator_fail(TaskCoordinator *coordinator,
                           TaskFailure failure,
                           uint32_t now_ms);
#ifdef __cplusplus
}
#endif
