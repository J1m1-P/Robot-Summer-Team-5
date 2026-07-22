/** @file task_executor.c
 *  @brief Implements single-task execution and lifecycle transitions.
 */
#include "task/task_executor.h"

#include <stddef.h>
#include <string.h>

#include "task/task_runner.h"

/**
 * @brief Dispatches the current step to its task-specific runner.
 * @param task Active task and its immutable request.
 * @return Current execution result of the step.
 */
static TaskStepResult run_current_step(const Task *task) {
    switch (task->request.type) {
        case TASK_TYPE_TOWER_PICKING:
        case TASK_TYPE_TOWER_BUILDING:
            return tower_task_run_step(task->request.type,
                                       task->current_step);
        case TASK_TYPE_TAPE_FOLLOWING:
            return tape_task_run_step(
                task->current_step,
                &task->request.params.tape_following);
        default:
            return TASK_STEP_FAILED;
    }
}

/**
 * @brief Advances the active task or completes its final step.
 * @param executor Executor containing the active task.
 * @return true when the task was advanced or completed.
 */
static bool advance_step(TaskExecutor *executor) {
    uint16_t step_count;
    if (!task_get_step_count(executor->task.request.type, &step_count)) {
        return false;
    }

    const uint16_t next_step = executor->task.current_step + 1U;
    if (next_step >= step_count) {
        executor->task.state = TASK_STATE_COMPLETED;
        return true;
    }

    TaskOwner next_owner;
    if (!task_get_step_owner(executor->task.request.type, next_step,
                             &next_owner)) {
        return false;
    }

    executor->task.current_step = next_step;
    executor->task.owner = next_owner;
    return true;
}

/**
 * @brief Resets a task executor.
 * @param executor Executor to initialize.
 */
void task_executor_init(TaskExecutor *executor) {
    if (executor == NULL) return;

    memset(executor, 0, sizeof(*executor));
}

/**
 * @brief Starts a task by copying its request.
 * @param executor Executor that will run the task.
 * @param request Task workflow and parameters to copy.
 * @return true when the task was started.
 */
bool task_executor_start(TaskExecutor *executor,
                         const TaskRequest *request) {
    if (executor == NULL || !task_request_is_valid(request) ||
        (executor->has_task &&
         executor->task.state == TASK_STATE_ACTIVE)) {
        return false;
    }

    TaskOwner owner;
    if (!task_get_step_owner(request->type, 0U, &owner)) return false;

    executor->task.request = *request;
    executor->task.owner = owner;
    executor->task.current_step = 0U;
    executor->task.state = TASK_STATE_ACTIVE;
    executor->has_task = true;
    return true;
}

/**
 * @brief Runs one update of the active task.
 * @param executor Executor to update.
 */
void task_executor_update(TaskExecutor *executor) {
    if (executor == NULL || !executor->has_task ||
        executor->task.state != TASK_STATE_ACTIVE) {
        return;
    }

    const TaskStepResult result = run_current_step(&executor->task);
    if (result == TASK_STEP_SUCCEEDED) {
        if (!advance_step(executor)) {
            executor->task.state = TASK_STATE_FAULTED;
        }
    } else if (result == TASK_STEP_FAILED) {
        executor->task.state = TASK_STATE_FAULTED;
    }
}

/**
 * @brief Cancels the active task.
 * @param executor Executor containing the active task.
 * @return true when the task was cancelled.
 */
bool task_executor_cancel(TaskExecutor *executor) {
    if (executor == NULL || !executor->has_task ||
        executor->task.state != TASK_STATE_ACTIVE) {
        return false;
    }

    executor->task.state = TASK_STATE_CANCELLED;
    return true;
}

/**
 * @brief Gets the active or most recently ended task.
 * @param executor Executor to query.
 * @return Task data, or NULL when no task has been started.
 */
const Task *task_executor_get_task(const TaskExecutor *executor) {
    if (executor == NULL || !executor->has_task) return NULL;

    return &executor->task;
}
