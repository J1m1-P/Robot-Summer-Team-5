/** @file task_executor.h
 *  @brief Executes one task request through its complete lifecycle.
 */
#pragma once

#include <stdbool.h>

#include <robot_common/task/task.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Stores one active or most recently ended task. */
typedef struct {
    Task task;     /**< Task request and runtime state. */
    bool has_task; /**< Whether task contains valid data. */
} TaskExecutor;

/**
 * @brief Resets a task executor.
 * @param executor Executor to initialize.
 */
void task_executor_init(TaskExecutor *executor);

/**
 * @brief Starts a task by copying its request.
 * @param executor Executor that will run the task.
 * @param request Task workflow and parameters to copy.
 * @return true when the task was started.
 */
bool task_executor_start(TaskExecutor *executor,
                         const TaskRequest *request);

/**
 * @brief Runs one update of the active task.
 * @param executor Executor to update.
 */
void task_executor_update(TaskExecutor *executor);

/**
 * @brief Cancels the active task.
 * @param executor Executor containing the active task.
 * @return true when the task was cancelled.
 */
bool task_executor_cancel(TaskExecutor *executor);

/**
 * @brief Gets the active or most recently ended task.
 * @param executor Executor to query.
 * @return Task data, or NULL when no task has been started.
 */
const Task *task_executor_get_task(const TaskExecutor *executor);

#ifdef __cplusplus
}
#endif
