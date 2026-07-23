/**
 * @file build_tower_action.h
 * @brief Declares the future nonblocking tower-building action boundary.
 *
 * The coordinator and arm manager can already route BUILD_TOWER commands here,
 * but the physical mechanism sequence is intentionally a terminal
 * NOT_IMPLEMENTED placeholder until its hardware behavior is defined.
 */
#pragma once

#include <robot_common/task/task_action_executor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TaskActionResult result; /**< Current placeholder action state. */
} BuildTowerAction;

/** Initializes the placeholder action in NOT_STARTED state. */
void build_tower_action_init(BuildTowerAction *action);
/** Accepts BUILD_TOWER and currently returns a NOT_IMPLEMENTED result. */
bool build_tower_action_start(BuildTowerAction *action,
                              const TaskStepCommand *command,
                              uint32_t now_ms);
/** Reserved nonblocking update hook for the future mechanism implementation. */
void build_tower_action_update(BuildTowerAction *action, uint32_t now_ms);
/** Cancels a future running build action; currently no placeholder run persists. */
bool build_tower_action_cancel(BuildTowerAction *action);
/** Test/harness hook that forces a running build action to succeed. */
bool build_tower_action_report_succeeded(BuildTowerAction *action);
/** Test/harness hook that fails a running build action with a reason. */
bool build_tower_action_report_failed(BuildTowerAction *action,
                                      TaskFailure failure);

#ifdef __cplusplus
}
#endif
