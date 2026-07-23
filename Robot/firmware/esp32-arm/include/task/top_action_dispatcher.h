/**
 * @file top_action_dispatcher.h
 * @brief Routes top-owned commands to the arm mechanism or Raspberry Pi.
 *
 * The drivetrain sees one top executor, while this dispatcher chooses whether
 * the requested action runs locally through ArmManager or remotely through the
 * Pi task-link client. It permits only one active target at a time.
 */
#pragma once

#include <robot_common/task/task_action_executor.h>

#include "task/arm_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TaskActionExecutor arm; /**< Local arm-mechanism executor. */
    TaskActionExecutor pi_scan; /**< Remote Raspberry Pi scan executor. */
    TaskActionExecutor *active; /**< Selected target, or NULL while idle. */
} TopActionDispatcher;

/** Initializes the dispatcher from a local arm manager and Pi executor. */
bool top_action_dispatcher_init(TopActionDispatcher *dispatcher,
                                ArmManager *arm_manager,
                                const TaskActionExecutor *pi_scan_executor);
/** Exposes routing callbacks for use by the drivetrain-facing task server. */
TaskActionExecutor top_action_dispatcher_executor(
    TopActionDispatcher *dispatcher);

#ifdef __cplusplus
}
#endif
