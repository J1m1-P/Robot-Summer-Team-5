/** @file task_coordinator.c
 *  @brief Implements deterministic task and step state transitions.
 */
#include "task/task_coordinator.h"

#include <stddef.h>
#include <string.h>

static bool task_is_running(const TaskCoordinator *coordinator) {
    return coordinator != NULL &&
           coordinator->runtime.status == TASK_STATUS_RUNNING;
}

static bool elapsed_at_least(uint32_t now, uint32_t then, uint32_t duration) {
    return (uint32_t)(now - then) >= duration;
}

static TaskActionExecutor *current_executor(TaskCoordinator *coordinator,
                                            TaskStepDefinition *step_out) {
    TaskStepDefinition step = {0};
    if (!task_get_step_definition(coordinator->runtime.request.type,
                                  coordinator->runtime.current_step, &step) ||
        !task_owner_is_valid(step.owner)) {
        return NULL;
    }
    if (step_out != NULL) *step_out = step;
    return &coordinator->executors[step.owner];
}

static void finish_failed(TaskCoordinator *coordinator,
                          TaskFailure failure,
                          uint32_t now_ms,
                          bool cancel_executor) {
    TaskActionExecutor *executor = current_executor(coordinator, NULL);
    if (cancel_executor && coordinator->runtime.step_status == TASK_STEP_RUNNING &&
        task_action_executor_is_valid(executor)) {
        executor->cancel(executor->context, now_ms);
    }
    coordinator->runtime.step_status = TASK_STEP_FAILED;
    coordinator->runtime.status = TASK_STATUS_FAILED;
    coordinator->runtime.failure = failure == TASK_FAILURE_NONE
                                       ? TASK_FAILURE_STEP_FAILED
                                       : failure;
}

bool task_coordinator_init(TaskCoordinator *coordinator,
                           const TaskCoordinatorConfig *config,
                           const TaskActionExecutor *drivetrain_executor,
                           const TaskActionExecutor *arm_executor) {
    if (coordinator == NULL || config == NULL ||
        config->step_timeout_ms == 0U ||
        !task_action_executor_is_valid(drivetrain_executor) ||
        !task_action_executor_is_valid(arm_executor)) {
        return false;
    }
    memset(coordinator, 0, sizeof(*coordinator));
    coordinator->config = *config;
    coordinator->executors[TASK_OWNER_DRIVETRAIN] = *drivetrain_executor;
    coordinator->executors[TASK_OWNER_ARM] = *arm_executor;
    coordinator->runtime.status = TASK_STATUS_IDLE;
    coordinator->runtime.step_status = TASK_STEP_NOT_STARTED;
    return true;
}

bool task_coordinator_start(TaskCoordinator *coordinator,
                            const TaskRequest *request,
                            uint32_t execution_id,
                            uint32_t now_ms) {
    (void)now_ms;
    if (coordinator == NULL || request == NULL || execution_id == 0U ||
        !task_request_is_valid(request) || task_is_running(coordinator)) {
        return false;
    }
    memset(&coordinator->runtime, 0, sizeof(coordinator->runtime));
    coordinator->runtime.execution_id = execution_id;
    coordinator->runtime.request = *request;
    coordinator->runtime.status = TASK_STATUS_RUNNING;
    coordinator->runtime.step_status = TASK_STEP_NOT_STARTED;
    coordinator->runtime.failure = TASK_FAILURE_NONE;
    return true;
}

void task_coordinator_update(TaskCoordinator *coordinator, uint32_t now_ms) {
    if (!task_is_running(coordinator)) return;

    TaskActionExecutor *executor = current_executor(coordinator, NULL);
    if (!task_action_executor_is_valid(executor)) {
        finish_failed(coordinator, TASK_FAILURE_INVALID_STEP, now_ms, false);
        return;
    }

    if (coordinator->runtime.step_status == TASK_STEP_NOT_STARTED) {
        TaskStepCommand command = {0};
        if (!task_build_step_command(&coordinator->runtime, &command) ||
            !executor->start(executor->context, &command, now_ms)) {
            finish_failed(coordinator, TASK_FAILURE_STEP_REJECTED,
                          now_ms, false);
            return;
        }
        coordinator->runtime.step_status = TASK_STEP_RUNNING;
        coordinator->step_started_ms = now_ms;
    }

    if (elapsed_at_least(now_ms, coordinator->step_started_ms,
                         coordinator->config.step_timeout_ms)) {
        finish_failed(coordinator, TASK_FAILURE_STEP_TIMEOUT, now_ms, true);
        return;
    }

    const TaskActionResult result =
        executor->update(executor->context, now_ms);
    switch (result.status) {
        case TASK_STEP_RUNNING:
            return;
        case TASK_STEP_SUCCEEDED: {
            uint8_t step_count = 0U;
            coordinator->runtime.step_status = TASK_STEP_SUCCEEDED;
            if (!task_get_step_count(coordinator->runtime.request.type,
                                     &step_count) || step_count == 0U) {
                finish_failed(coordinator, TASK_FAILURE_INVALID_STEP,
                              now_ms, false);
                return;
            }
            const uint8_t next_step =
                (uint8_t)(coordinator->runtime.current_step + 1U);
            if (next_step >= step_count) {
                coordinator->runtime.status = TASK_STATUS_SUCCEEDED;
                coordinator->runtime.failure = TASK_FAILURE_NONE;
                return;
            }
            coordinator->runtime.current_step = next_step;
            coordinator->runtime.step_status = TASK_STEP_NOT_STARTED;
            return;
        }
        case TASK_STEP_FAILED:
            finish_failed(coordinator, result.failure, now_ms, false);
            return;
        case TASK_STEP_CANCELLED:
            coordinator->runtime.step_status = TASK_STEP_CANCELLED;
            coordinator->runtime.status = TASK_STATUS_CANCELLED;
            coordinator->runtime.failure = TASK_FAILURE_NONE;
            return;
        default:
            finish_failed(coordinator, TASK_FAILURE_PROTOCOL, now_ms, true);
            return;
    }
}

bool task_coordinator_cancel(TaskCoordinator *coordinator, uint32_t now_ms) {
    if (!task_is_running(coordinator)) return false;
    TaskActionExecutor *executor = current_executor(coordinator, NULL);
    if (coordinator->runtime.step_status == TASK_STEP_RUNNING &&
        task_action_executor_is_valid(executor)) {
        executor->cancel(executor->context, now_ms);
    }
    coordinator->runtime.step_status = TASK_STEP_CANCELLED;
    coordinator->runtime.status = TASK_STATUS_CANCELLED;
    coordinator->runtime.failure = TASK_FAILURE_NONE;
    return true;
}

bool task_coordinator_fail(TaskCoordinator *coordinator,
                           TaskFailure failure,
                           uint32_t now_ms) {
    if (!task_is_running(coordinator) || failure == TASK_FAILURE_NONE) {
        return false;
    }
    finish_failed(coordinator, failure, now_ms, true);
    return true;
}

const TaskRuntime *task_coordinator_get_runtime(
    const TaskCoordinator *coordinator) {
    return coordinator == NULL ? NULL : &coordinator->runtime;
}

bool task_coordinator_get_current_owner(const TaskCoordinator *coordinator,
                                        TaskOwner *owner_out) {
    if (coordinator == NULL || owner_out == NULL ||
        coordinator->runtime.status != TASK_STATUS_RUNNING) {
        return false;
    }
    TaskStepDefinition step = {0};
    if (!task_get_step_definition(coordinator->runtime.request.type,
                                  coordinator->runtime.current_step, &step)) {
        return false;
    }
    *owner_out = step.owner;
    return true;
}
