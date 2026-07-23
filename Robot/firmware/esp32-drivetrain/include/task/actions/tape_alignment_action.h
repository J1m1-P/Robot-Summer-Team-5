/** @file tape_alignment_action.h
 *  @brief Shared implementation of the two tape-alignment actions.
 */
#pragma once

#include "control/drivetrain/drivetrain.h"
#include "control/tape_following/tape_follower.h"
#include "control/tape_following/tape_following_controller.h"
#include "drivers/tape_sensor/tape_sensor_driver.h"
#include "sensing/tape_following/tape_line_estimator.h"
#include "task/drivetrain_action_handler.h"
#include <robot_common/task/task_action_executor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Drivetrain *drivetrain;
    TapeSensor *front_sensor;
    TapeSensor *back_sensor;
    TapeSensor *left_sensor;
    TapeFollower *tape_follower;
    TaskActionResult result;
    TapeFollowerSensor selected_sensor;
    TapeLineEstimatorState estimator_state;
    TapeFollowingControllerState controller_state;
    uint32_t last_update_ms;
    uint8_t stable_samples;
} TapeAlignmentAction;

void tape_alignment_action_init(TapeAlignmentAction *action,
                                Drivetrain *drivetrain,
                                TapeSensor *front_sensor,
                                TapeSensor *back_sensor,
                                TapeSensor *left_sensor,
                                TapeFollower *tape_follower);
bool tape_alignment_action_start(TapeAlignmentAction *action,
                                 const TaskStepCommand *command,
                                 uint32_t now_ms);
TaskActionResult tape_alignment_action_update(TapeAlignmentAction *action,
                                              uint32_t now_ms);
void tape_alignment_action_cancel(TapeAlignmentAction *action);
bool tape_alignment_action_report_succeeded(TapeAlignmentAction *action);
bool tape_alignment_action_report_failed(TapeAlignmentAction *action,
                                         TaskFailure failure);
DrivetrainActionHandler tape_alignment_action_handler(
    TapeAlignmentAction *action);

#ifdef __cplusplus
}
#endif
