/** @file drivetrain_action_handler.h
 *  @brief Interface implemented by one drivetrain task-action module.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/task/task_action_executor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef bool (*DrivetrainActionReportSucceeded)(void *context);
typedef bool (*DrivetrainActionReportFailed)(void *context,
                                             TaskFailure failure);

/**
 * One independently implemented drivetrain action module.
 *
 * supported_actions is a TASK_ACTION_BIT mask. Multiple action IDs may share
 * one implementation when their command parameters select the behavior.
 */
typedef struct {
    uint32_t supported_actions;
    TaskActionExecutor executor;
    DrivetrainActionReportSucceeded report_succeeded;
    DrivetrainActionReportFailed report_failed;
} DrivetrainActionHandler;

#ifdef __cplusplus
}
#endif
