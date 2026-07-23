/** @file top_action_dispatcher.h
 *  @brief Selects the top-local executor for one requested action.
 */
#pragma once

#include <robot_common/task/task_action_executor.h>

#include "task/arm_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TaskActionExecutor arm;
    TaskActionExecutor pi_scan;
    TaskActionExecutor *active;
} TopActionDispatcher;

bool top_action_dispatcher_init(TopActionDispatcher *dispatcher,
                                ArmManager *arm_manager,
                                const TaskActionExecutor *pi_scan_executor);
TaskActionExecutor top_action_dispatcher_executor(
    TopActionDispatcher *dispatcher);

#ifdef __cplusplus
}
#endif
