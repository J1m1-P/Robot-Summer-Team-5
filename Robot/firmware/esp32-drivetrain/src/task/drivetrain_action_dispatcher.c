#include "task/drivetrain_action_dispatcher.h"

#include <stddef.h>
#include <string.h>

static bool executor_is_valid(const TaskActionExecutor *executor) {
    return executor != NULL && executor->context != NULL &&
           executor->start != NULL && executor->update != NULL &&
           executor->cancel != NULL;
}

void drivetrain_action_dispatcher_init(
    DrivetrainActionDispatcher *dispatcher) {
    if (dispatcher == NULL) return;
    memset(dispatcher, 0, sizeof(*dispatcher));
}

bool drivetrain_action_dispatcher_register(
    DrivetrainActionDispatcher *dispatcher,
    const DrivetrainActionHandler *handler) {
    if (dispatcher == NULL || handler == NULL ||
        handler->supported_actions == 0U ||
        !executor_is_valid(&handler->executor) ||
        dispatcher->handler_count >= DRIVETRAIN_ACTION_HANDLER_CAPACITY) {
        return false;
    }

    for (size_t i = 0; i < dispatcher->handler_count; ++i) {
        if ((dispatcher->handlers[i].supported_actions &
             handler->supported_actions) != 0U) {
            return false;
        }
    }

    dispatcher->handlers[dispatcher->handler_count++] = *handler;
    return true;
}

static DrivetrainActionHandler *find_handler(
    DrivetrainActionDispatcher *dispatcher, TaskAction action) {
    if (dispatcher == NULL || !task_action_is_valid(action)) return NULL;
    const uint32_t action_bit = TASK_ACTION_BIT(action);
    for (size_t i = 0; i < dispatcher->handler_count; ++i) {
        if ((dispatcher->handlers[i].supported_actions & action_bit) != 0U) {
            return &dispatcher->handlers[i];
        }
    }
    return NULL;
}

static bool dispatcher_start(void *context, const TaskStepCommand *command,
                             uint32_t now_ms) {
    DrivetrainActionDispatcher *dispatcher =
        (DrivetrainActionDispatcher *)context;
    if (dispatcher == NULL || command == NULL || dispatcher->active != NULL) {
        return false;
    }

    DrivetrainActionHandler *handler =
        find_handler(dispatcher, command->action);
    if (handler == NULL ||
        !handler->executor.start(handler->executor.context, command, now_ms)) {
        return false;
    }
    dispatcher->active = handler;
    return true;
}

static TaskActionResult dispatcher_update(void *context, uint32_t now_ms) {
    DrivetrainActionDispatcher *dispatcher =
        (DrivetrainActionDispatcher *)context;
    if (dispatcher == NULL || dispatcher->active == NULL) {
        return (TaskActionResult){TASK_STEP_FAILED, TASK_FAILURE_PROTOCOL};
    }

    const TaskActionResult result = dispatcher->active->executor.update(
        dispatcher->active->executor.context, now_ms);
    if (task_step_status_is_terminal(result.status)) {
        dispatcher->active = NULL;
    }
    return result;
}

static void dispatcher_cancel(void *context, uint32_t now_ms) {
    DrivetrainActionDispatcher *dispatcher =
        (DrivetrainActionDispatcher *)context;
    if (dispatcher == NULL || dispatcher->active == NULL) return;
    dispatcher->active->executor.cancel(
        dispatcher->active->executor.context, now_ms);
    dispatcher->active = NULL;
}

TaskActionExecutor drivetrain_action_dispatcher_executor(
    DrivetrainActionDispatcher *dispatcher) {
    return (TaskActionExecutor){
        .context = dispatcher,
        .start = dispatcher_start,
        .update = dispatcher_update,
        .cancel = dispatcher_cancel,
    };
}

bool drivetrain_action_dispatcher_report_succeeded(
    DrivetrainActionDispatcher *dispatcher) {
    return dispatcher != NULL && dispatcher->active != NULL &&
           dispatcher->active->report_succeeded != NULL &&
           dispatcher->active->report_succeeded(
               dispatcher->active->executor.context);
}

bool drivetrain_action_dispatcher_report_failed(
    DrivetrainActionDispatcher *dispatcher, TaskFailure failure) {
    return dispatcher != NULL && dispatcher->active != NULL &&
           dispatcher->active->report_failed != NULL &&
           dispatcher->active->report_failed(
               dispatcher->active->executor.context, failure);
}
