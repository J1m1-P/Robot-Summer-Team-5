/**
 * @file follow_tape_action.h
 * @brief Declares the nonblocking route-following drivetrain action.
 *
 * This action reads the tape modules, runs the tape follower, commands body
 * velocity, and integrates encoder odometry until the requested path length is
 * reached. It owns per-run control state but borrows shared hardware objects.
 */
#pragma once

#include "control/drivetrain/drivetrain.h"
#include "control/drivetrain/drivetrain_odometry_source.h"
#include "control/drivetrain/odometry.h"
#include "control/tape_following/tape_follower.h"
#include "drivers/tape_sensor/tape_sensor_driver.h"
#include <robot_common/task/task_action_executor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Drivetrain *drivetrain; /**< Borrowed body-velocity controller. */
    TapeSensor *front_sensor; /**< Leading/forward guidance module. */
    TapeSensor *back_sensor; /**< Rear/reverse guidance module. */
    TapeSensor *left_sensor; /**< Sampled with the shared three-module reader. */
    TapeFollower *tape_follower; /**< Borrowed line-following controller. */
    TaskActionResult result; /**< Latest start/update/cancel result. */
    DrivetrainOdometry odometry; /**< Pose accumulated during this action. */
    DrivetrainOdometrySource odometry_source; /**< Encoder baseline state. */
    DrivetrainOdometrySourceConfig odometry_source_config; /**< Wheel geometry. */
    uint32_t last_update_ms; /**< Timestamp used to compute controller dt. */
    float signed_travel_speed_mps; /**< Direction-applied speed command. */
    float target_distance_m; /**< Path length required for success. */
    float traveled_distance_m; /**< Integrated path length so far. */
} FollowTapeAction;

/** Binds shared drivetrain/sensor/controller dependencies and leaves action idle. */
void follow_tape_action_init(FollowTapeAction *action, Drivetrain *drivetrain,
                             TapeSensor *front_sensor,
                             TapeSensor *back_sensor,
                             TapeSensor *left_sensor,
                             TapeFollower *tape_follower);
/** Validates a FOLLOW_TAPE command and initializes one nonblocking run. */
bool follow_tape_action_start(FollowTapeAction *action,
                              const TaskStepCommand *command,
                              uint32_t now_ms);
/** Reads sensors, commands motion, and returns running/success/failure state. */
TaskActionResult follow_tape_action_update(FollowTapeAction *action,
                                           uint32_t now_ms);
/** Brakes the drivetrain and marks a running action cancelled. */
void follow_tape_action_cancel(FollowTapeAction *action);
/** Test/harness hook that completes a currently running action successfully. */
bool follow_tape_action_report_succeeded(FollowTapeAction *action);
/** Test/harness hook that fails a running action with the supplied reason. */
bool follow_tape_action_report_failed(FollowTapeAction *action,
                                      TaskFailure failure);

#ifdef __cplusplus
}
#endif
