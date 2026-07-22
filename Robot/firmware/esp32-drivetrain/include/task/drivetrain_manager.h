/** @file drivetrain_manager.h
 *  @brief Executes drivetrain-owned actions without sequencing tasks.
 */
#pragma once

#include "control/drivetrain/drivetrain.h"
#include "task/task_action_executor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Drivetrain *drivetrain;
    TaskActionResult result;
} DrivetrainManager;

void drivetrain_manager_init(DrivetrainManager *manager,
                             Drivetrain *drivetrain);
TaskActionExecutor drivetrain_manager_executor(DrivetrainManager *manager);
bool drivetrain_manager_report_succeeded(DrivetrainManager *manager);
bool drivetrain_manager_report_failed(DrivetrainManager *manager,
                                      TaskFailure failure);

#ifdef __cplusplus
}
#endif
