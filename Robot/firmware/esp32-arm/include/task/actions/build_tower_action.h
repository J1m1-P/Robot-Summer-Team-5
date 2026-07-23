/** @file build_tower_action.h
 *  @brief Stateful implementation boundary for TASK_ACTION_BUILD_TOWER.
 */
#pragma once

#include <robot_common/task/task_action_executor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TaskActionResult result;
} BuildTowerAction;

void build_tower_action_init(BuildTowerAction *action);
bool build_tower_action_start(BuildTowerAction *action,
                              const TaskStepCommand *command,
                              uint32_t now_ms);
void build_tower_action_update(BuildTowerAction *action, uint32_t now_ms);
bool build_tower_action_cancel(BuildTowerAction *action);
bool build_tower_action_report_succeeded(BuildTowerAction *action);
bool build_tower_action_report_failed(BuildTowerAction *action,
                                      TaskFailure failure);

#ifdef __cplusplus
}
#endif
