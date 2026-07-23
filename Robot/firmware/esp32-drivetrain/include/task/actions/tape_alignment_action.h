/** @file tape_alignment_action.h
 *  @brief Shared implementation of the two tape-alignment actions.
 */
#pragma once

#include "control/drivetrain/drivetrain.h"
#include "control/drivetrain/drivetrain_odometry_source.h"
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
    TaskAction active_action;
    TaskStepParameters parameters;
    TapeLineEstimatorState estimator_state;
    TapeFollowingControllerState controller_state;
    DrivetrainOdometrySource odometry_source;
    DrivetrainOdometrySourceConfig odometry_source_config;
    DrivetrainOdometry odometry;
    uint32_t last_update_ms;
    float traveled_distance_m;
    uint8_t phase;
    uint8_t stable_samples;
    volatile bool task_tape_detected;
    volatile bool route_tape_detected;
    bool route_tape_cleared;
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
/** Tiny flag-only hooks for the future detector ISR. */
static inline __attribute__((always_inline)) void
tape_alignment_action_notify_task_tape(TapeAlignmentAction *action) {
    if (action != NULL) action->task_tape_detected = true;
}
static inline __attribute__((always_inline)) void
tape_alignment_action_notify_route_tape(TapeAlignmentAction *action) {
    if (action != NULL) action->route_tape_detected = true;
}
bool tape_alignment_action_report_succeeded(TapeAlignmentAction *action);
bool tape_alignment_action_report_failed(TapeAlignmentAction *action,
                                         TaskFailure failure);
DrivetrainActionHandler tape_alignment_action_handler(
    TapeAlignmentAction *action);

#ifdef __cplusplus
}
#endif
