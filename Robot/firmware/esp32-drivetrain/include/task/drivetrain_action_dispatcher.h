/** @file drivetrain_action_dispatcher.h
 *  @brief Routes commands to registered drivetrain action modules.
 */
#pragma once

#include <stddef.h>

#include "task/drivetrain_action_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRIVETRAIN_ACTION_HANDLER_CAPACITY 8U

typedef struct {
    DrivetrainActionHandler handlers[DRIVETRAIN_ACTION_HANDLER_CAPACITY];
    size_t handler_count;
    DrivetrainActionHandler *active;
} DrivetrainActionDispatcher;

void drivetrain_action_dispatcher_init(
    DrivetrainActionDispatcher *dispatcher);

/**
 * Registers a handler. Registration fails for an invalid handler, an action
 * mask that overlaps an existing handler, or a full dispatcher.
 */
bool drivetrain_action_dispatcher_register(
    DrivetrainActionDispatcher *dispatcher,
    const DrivetrainActionHandler *handler);

TaskActionExecutor drivetrain_action_dispatcher_executor(
    DrivetrainActionDispatcher *dispatcher);
bool drivetrain_action_dispatcher_report_succeeded(
    DrivetrainActionDispatcher *dispatcher);
bool drivetrain_action_dispatcher_report_failed(
    DrivetrainActionDispatcher *dispatcher, TaskFailure failure);

#ifdef __cplusplus
}
#endif
