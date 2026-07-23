/**
 * @file task_coordinator.h
 * @brief Declares the sole owner of workflow sequencing and task runtime state.
 *
 * The coordinator selects immutable workflow steps, sends each step to its
 * drivetrain or top executor, and advances only on success. It also centralizes
 * timeout, cancellation, failure, and final safe-state behavior.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/task/task_action_executor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t step_timeout_ms; /**< Maximum runtime allowed for one step. */
} TaskCoordinatorConfig;

typedef bool (*TaskEnterSafeState)(void *context);

typedef struct {
    void *context; /**< Hardware state supplied to enter(). */
    TaskEnterSafeState enter; /**< Callback that stops the robot safely. */
} TaskSafeStateHandler;

typedef struct {
    TaskCoordinatorConfig config; /**< Validated timeout policy. */
    TaskActionExecutor executors[TASK_OWNER_COUNT]; /**< Executor per owner. */
    TaskSafeStateHandler safe_state; /**< Final stop/brake operation. */
    TaskRuntime runtime; /**< Authoritative current task and step state. */
    uint32_t step_started_ms; /**< Timestamp used for step timeout. */
} TaskCoordinator;

/** Initializes an idle coordinator with local/remote executors and safe state. */
bool task_coordinator_init(TaskCoordinator *coordinator,
                           const TaskCoordinatorConfig *config,
                           const TaskActionExecutor *drivetrain_executor,
                           const TaskActionExecutor *top_executor,
                           const TaskSafeStateHandler *safe_state);
/** Copies a valid request into a new nonzero execution and marks it running. */
bool task_coordinator_start(TaskCoordinator *coordinator,
                            const TaskRequest *request,
                            uint32_t execution_id);
/** Starts or polls the current step and performs all lifecycle transitions. */
void task_coordinator_update(TaskCoordinator *coordinator, uint32_t now_ms);
/** Cancels the active executor and enters the configured safe state. */
bool task_coordinator_cancel(TaskCoordinator *coordinator, uint32_t now_ms);
/** Injects a subsystem failure through the coordinator's fail-safe path. */
bool task_coordinator_fail(TaskCoordinator *coordinator,
                           TaskFailure failure,
                           uint32_t now_ms);
#ifdef __cplusplus
}
#endif
