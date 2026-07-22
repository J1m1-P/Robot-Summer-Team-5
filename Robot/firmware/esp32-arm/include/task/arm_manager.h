/** @file arm_manager.h
 *  @brief Executes one arm-owned action without sequencing robot tasks.
 */
#pragma once

#include <stdbool.h>

#include <robot_common/task/task.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TaskStepCommand command;
    TaskStepStatus status;
    TaskFailure failure;
    bool active;
} ArmManager;

void arm_manager_init(ArmManager *manager);
bool arm_manager_start(ArmManager *manager, const TaskStepCommand *command);
void arm_manager_update(ArmManager *manager);
bool arm_manager_cancel(ArmManager *manager);
bool arm_manager_report_succeeded(ArmManager *manager);
bool arm_manager_report_failed(ArmManager *manager, TaskFailure failure);
TaskStepStatus arm_manager_get_status(const ArmManager *manager,
                                      TaskFailure *failure_out);

#ifdef __cplusplus
}
#endif
