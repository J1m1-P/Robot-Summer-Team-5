/**
 * @file tape_alignment_action.h
 * @brief Declares the shared implementation for picking-route drive actions.
 *
 * One state machine handles rotation/alignment, tape-following approach,
 * distance-controlled task-tape travel, backing away, and route reacquisition.
 * The command's TaskAction selects the behavior while state remains isolated
 * from the workflow coordinator.
 */
#pragma once

#include "control/drivetrain/drivetrain.h"
#include "control/drivetrain/drivetrain_odometry_source.h"
#include "control/tape_following/tape_follower.h"
#include "control/tape_following/tape_following_controller.h"
#include "drivers/tape_sensor/tape_sensor_driver.h"
#include "sensing/tape_following/tape_line_estimator.h"
#include <robot_common/task/task_action_executor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Drivetrain *drivetrain; /**< Borrowed body-velocity controller. */
    TapeSensor *front_sensor; /**< Front route-following module. */
    TapeSensor *back_sensor; /**< Back task-tape module. */
    TapeSensor *left_sensor; /**< Side guidance and alignment module. */
    TapeFollower *tape_follower; /**< Borrowed line-following controller. */
    TaskActionResult result; /**< Latest state returned to the manager. */
    TaskAction active_action; /**< Which supported behavior is running. */
    TaskStepParameters parameters; /**< Amount/speed copied from the command. */
    TapeLineEstimatorState estimator_state; /**< Left-line estimator history. */
    TapeFollowingControllerState controller_state; /**< Alignment PID state. */
    DrivetrainOdometrySource odometry_source; /**< Encoder baseline state. */
    DrivetrainOdometrySourceConfig odometry_source_config; /**< Wheel geometry. */
    DrivetrainOdometry odometry; /**< Pose accumulated during this action. */
    uint32_t last_update_ms; /**< Timestamp used to compute controller dt. */
    float traveled_distance_m; /**< Integrated path length for distance gates. */
    uint8_t phase; /**< Internal primary/seek/align state. */
    uint8_t stable_samples; /**< Consecutive centered readings. */
    volatile bool task_tape_detected; /**< Latched task-tape event. */
    volatile bool route_tape_detected; /**< Latched main-route event. */
    bool route_tape_cleared; /**< Ensures route reacquisition sees a new edge. */
} TapeAlignmentAction;

/** Binds shared drive/sensor/controller dependencies and leaves action idle. */
void tape_alignment_action_init(TapeAlignmentAction *action,
                                Drivetrain *drivetrain,
                                TapeSensor *front_sensor,
                                TapeSensor *back_sensor,
                                TapeSensor *left_sensor,
                                TapeFollower *tape_follower);
/** Validates and starts one supported picking-route drive command. */
bool tape_alignment_action_start(TapeAlignmentAction *action,
                                 const TaskStepCommand *command,
                                 uint32_t now_ms);
/** Advances the selected behavior and returns its latest action result. */
TaskActionResult tape_alignment_action_update(TapeAlignmentAction *action,
                                              uint32_t now_ms);
/** Brakes the drivetrain and marks the active behavior cancelled. */
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
/** Test/harness hook that completes a running behavior successfully. */
bool tape_alignment_action_report_succeeded(TapeAlignmentAction *action);
/** Test/harness hook that fails a running behavior with the supplied reason. */
bool tape_alignment_action_report_failed(TapeAlignmentAction *action,
                                          TaskFailure failure);

#ifdef __cplusplus
}
#endif
