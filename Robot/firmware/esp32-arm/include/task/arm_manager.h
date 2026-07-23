/** @file arm_manager.h
 *  @brief Executes one arm-owned action without sequencing robot tasks.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/task/task_action_executor.h>
#include "task/actions/build_tower_action.h"
#include "task/actions/pick_up_block_action.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TaskAction active_action;
    PickUpBlockAction pick_up_block;
    BuildTowerAction build_tower;
} ArmManager;

void arm_manager_init(ArmManager *manager);
bool arm_manager_start(ArmManager *manager, const TaskStepCommand *command,
                       uint32_t now_ms);
void arm_manager_update(ArmManager *manager, uint32_t now_ms);
bool arm_manager_cancel(ArmManager *manager);
bool arm_manager_report_succeeded(ArmManager *manager);
bool arm_manager_report_failed(ArmManager *manager, TaskFailure failure);
TaskStepStatus arm_manager_get_status(const ArmManager *manager,
                                      TaskFailure *failure_out);
TaskActionExecutor arm_manager_executor(ArmManager *manager);

#ifdef __cplusplus
}
#endif
