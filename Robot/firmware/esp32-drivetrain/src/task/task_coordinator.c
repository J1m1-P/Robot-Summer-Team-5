/** @file task_coordinator.c
 *  @brief Implements deterministic task and step state transitions.
 */
#include "task/task_coordinator.h"

#include <stddef.h>
#include <string.h>

typedef struct {
    TaskAction action;
    TaskOwner owner;
} TaskStepDefinition;

typedef struct {
    const TaskStepDefinition *steps;
    uint8_t count;
} WorkflowDefinition;

static const TaskStepDefinition TAPE_WORKFLOW[] = {
    {TASK_ACTION_FOLLOW_TAPE, TASK_OWNER_DRIVETRAIN},
};

static const TaskStepDefinition PICKING_WORKFLOW[] = {
    {TASK_ACTION_ALIGN_TO_PIECES, TASK_OWNER_DRIVETRAIN},
    {TASK_ACTION_PICK_UP_BLOCK, TASK_OWNER_TOP},
    {TASK_ACTION_ALIGN_TO_TAPE, TASK_OWNER_DRIVETRAIN},
};

static const TaskStepDefinition BUILDING_WORKFLOW[] = {
    {TASK_ACTION_BUILD_TOWER, TASK_OWNER_TOP},
};

static const TaskStepDefinition SCAN_WORKFLOW[] = {
    {TASK_ACTION_SCAN_TELETUBBIES, TASK_OWNER_TOP},
};

// Keeps workflow lookup and representation private to the sole sequencing owner.
static bool get_workflow(TaskType type, WorkflowDefinition *workflow_out) {
    if (workflow_out == NULL) return false;

    switch (type) {
        case TASK_TYPE_TAPE_FOLLOWING:
            *workflow_out = (WorkflowDefinition){
                TAPE_WORKFLOW, sizeof(TAPE_WORKFLOW) / sizeof(TAPE_WORKFLOW[0])};
            return true;
        case TASK_TYPE_TOWER_PICKING:
            *workflow_out = (WorkflowDefinition){
                PICKING_WORKFLOW,
                sizeof(PICKING_WORKFLOW) / sizeof(PICKING_WORKFLOW[0])};
            return true;
        case TASK_TYPE_TOWER_BUILDING:
            *workflow_out = (WorkflowDefinition){
                BUILDING_WORKFLOW,
                sizeof(BUILDING_WORKFLOW) / sizeof(BUILDING_WORKFLOW[0])};
            return true;
        case TASK_TYPE_TELETUBBY_SCAN:
            *workflow_out = (WorkflowDefinition){
                SCAN_WORKFLOW,
                sizeof(SCAN_WORKFLOW) / sizeof(SCAN_WORKFLOW[0])};
            return true;
        default:
            return false;
    }
}

// Resolves one private workflow entry without exposing workflow internals publicly.
static bool get_step(TaskType type, uint8_t index,
                     TaskStepDefinition *step_out) {
    WorkflowDefinition workflow = {0};
    if (step_out == NULL || !get_workflow(type, &workflow) ||
        index >= workflow.count) {
        return false;
    }
    *step_out = workflow.steps[index];
    return true;
}

// Builds the current executor command from authoritative coordinator state.
static bool build_step_command(const TaskRuntime *runtime,
                               TaskStepCommand *command_out) {
    if (runtime == NULL || command_out == NULL ||
        runtime->status != TASK_STATUS_RUNNING) {
        return false;
    }

    TaskStepDefinition step = {0};
    if (!get_step(runtime->request.type, runtime->current_step, &step)) {
        return false;
    }

    memset(command_out, 0, sizeof(*command_out));
    command_out->execution_id = runtime->execution_id;
    command_out->step = runtime->current_step;
    command_out->action = step.action;
    if (step.action != TASK_ACTION_FOLLOW_TAPE) return true;

    if (runtime->request.type == TASK_TYPE_TAPE_FOLLOWING) {
        command_out->tape_following = runtime->request.params.tape_following;
        return true;
    }
    return false;
}

// Validates the callback table at the one module that invokes it.
static bool executor_is_valid(const TaskActionExecutor *executor) {
    return executor != NULL && executor->start != NULL &&
           executor->update != NULL && executor->cancel != NULL;
}

// Treats the runtime status as the sole authority for whether work may advance.
static bool task_is_running(const TaskCoordinator *coordinator) {
    return coordinator != NULL &&
           coordinator->runtime.status == TASK_STATUS_RUNNING;
}

// Performs wrap-safe millisecond deadline checks.
static bool elapsed_at_least(uint32_t now, uint32_t then, uint32_t duration) {
    return (uint32_t)(now - then) >= duration;
}

// Resolves the current immutable workflow step to its registered owner executor.
static TaskActionExecutor *current_executor(TaskCoordinator *coordinator,
                                            TaskStepDefinition *step_out) {
    TaskStepDefinition step = {0};
    if (!get_step(coordinator->runtime.request.type,
                  coordinator->runtime.current_step, &step) ||
        step.owner >= TASK_OWNER_COUNT) {
        return NULL;
    }
    if (step_out != NULL) *step_out = step;
    return &coordinator->executors[step.owner];
}

// Applies the common failed-task transition and optionally cancels active hardware.
static void finish_failed(TaskCoordinator *coordinator,
                          TaskFailure failure,
                          uint32_t now_ms,
                          bool cancel_executor) {
    TaskActionExecutor *executor = current_executor(coordinator, NULL);
    if (cancel_executor && coordinator->runtime.step_status == TASK_STEP_RUNNING &&
        executor_is_valid(executor)) {
        executor->cancel(executor->context, now_ms);
    }
    coordinator->runtime.step_status = TASK_STEP_FAILED;
    coordinator->runtime.status = TASK_STATUS_FAILED;
    coordinator->runtime.failure = failure == TASK_FAILURE_NONE
                                       ? TASK_FAILURE_STEP_FAILED
                                       : failure;
    if (!coordinator->safe_state.enter(coordinator->safe_state.context)) {
        coordinator->runtime.failure = TASK_FAILURE_SAFE_STATE_FAILED;
    }
}

static void finish_succeeded(TaskCoordinator *coordinator) {
    coordinator->runtime.step_status = TASK_STEP_SUCCEEDED;
    if (!coordinator->safe_state.enter(coordinator->safe_state.context)) {
        coordinator->runtime.status = TASK_STATUS_FAILED;
        coordinator->runtime.failure = TASK_FAILURE_SAFE_STATE_FAILED;
        return;
    }
    coordinator->runtime.status = TASK_STATUS_SUCCEEDED;
    coordinator->runtime.failure = TASK_FAILURE_NONE;
}

// Validates dependencies and creates one idle authoritative task runtime.
bool task_coordinator_init(TaskCoordinator *coordinator,
                           const TaskCoordinatorConfig *config,
                           const TaskActionExecutor *drivetrain_executor,
                           const TaskActionExecutor *top_executor,
                           const TaskSafeStateHandler *safe_state) {
    if (coordinator == NULL || config == NULL ||
        config->step_timeout_ms == 0U ||
        !executor_is_valid(drivetrain_executor) ||
        !executor_is_valid(top_executor) || safe_state == NULL ||
        safe_state->enter == NULL) {
        return false;
    }
    memset(coordinator, 0, sizeof(*coordinator));
    coordinator->config = *config;
    coordinator->executors[TASK_OWNER_DRIVETRAIN] = *drivetrain_executor;
    coordinator->executors[TASK_OWNER_TOP] = *top_executor;
    coordinator->safe_state = *safe_state;
    coordinator->runtime.status = TASK_STATUS_IDLE;
    coordinator->runtime.step_status = TASK_STEP_NOT_STARTED;
    return true;
}

// Accepts a validated request only when no workflow is already running.
bool task_coordinator_start(TaskCoordinator *coordinator,
                            const TaskRequest *request,
                            uint32_t execution_id) {
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

// Starts, monitors, times out, and advances exactly one workflow step per state transition.
void task_coordinator_update(TaskCoordinator *coordinator, uint32_t now_ms) {
    if (!task_is_running(coordinator)) return;

    TaskActionExecutor *executor = current_executor(coordinator, NULL);
    if (!executor_is_valid(executor)) {
        finish_failed(coordinator, TASK_FAILURE_INVALID_STEP, now_ms, false);
        return;
    }

    if (coordinator->runtime.step_status == TASK_STEP_NOT_STARTED) {
        TaskStepCommand command = {0};
        if (!build_step_command(&coordinator->runtime, &command) ||
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
            WorkflowDefinition workflow = {0};
            coordinator->runtime.step_status = TASK_STEP_SUCCEEDED;
            if (!get_workflow(coordinator->runtime.request.type, &workflow) ||
                workflow.count == 0U) {
                finish_failed(coordinator, TASK_FAILURE_INVALID_STEP,
                              now_ms, false);
                return;
            }
            const uint8_t next_step =
                (uint8_t)(coordinator->runtime.current_step + 1U);
            if (next_step >= workflow.count) {
                finish_succeeded(coordinator);
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
            if (!coordinator->safe_state.enter(coordinator->safe_state.context)) {
                coordinator->runtime.status = TASK_STATUS_FAILED;
                coordinator->runtime.failure = TASK_FAILURE_SAFE_STATE_FAILED;
            }
            return;
        default:
            finish_failed(coordinator, TASK_FAILURE_PROTOCOL, now_ms, true);
            return;
    }
}

// Cancels the active owner first, then marks the authoritative runtime cancelled.
bool task_coordinator_cancel(TaskCoordinator *coordinator, uint32_t now_ms) {
    if (!task_is_running(coordinator)) return false;
    TaskActionExecutor *executor = current_executor(coordinator, NULL);
    if (coordinator->runtime.step_status == TASK_STEP_RUNNING &&
        executor_is_valid(executor)) {
        executor->cancel(executor->context, now_ms);
    }
    coordinator->runtime.step_status = TASK_STEP_CANCELLED;
    coordinator->runtime.status = TASK_STATUS_CANCELLED;
    coordinator->runtime.failure = TASK_FAILURE_NONE;
    if (!coordinator->safe_state.enter(coordinator->safe_state.context)) {
        coordinator->runtime.status = TASK_STATUS_FAILED;
        coordinator->runtime.failure = TASK_FAILURE_SAFE_STATE_FAILED;
    }
    return true;
}

// Injects an external subsystem failure through the same fail-safe transition path.
bool task_coordinator_fail(TaskCoordinator *coordinator,
                           TaskFailure failure,
                           uint32_t now_ms) {
    if (!task_is_running(coordinator) || failure == TASK_FAILURE_NONE) {
        return false;
    }
    finish_failed(coordinator, failure, now_ms, true);
    return true;
}
