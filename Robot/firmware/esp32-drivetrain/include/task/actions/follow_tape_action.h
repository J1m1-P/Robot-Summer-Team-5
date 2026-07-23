/** @file follow_tape_action.h
 *  @brief Stateful implementation of TASK_ACTION_FOLLOW_TAPE.
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
    Drivetrain *drivetrain;
    TapeSensor *front_sensor;
    TapeSensor *back_sensor;
    TapeSensor *left_sensor;
    TapeFollower *tape_follower;
    TaskActionResult result;
    DrivetrainOdometry odometry;
    DrivetrainOdometrySource odometry_source;
    DrivetrainOdometrySourceConfig odometry_source_config;
    uint32_t last_update_ms;
    float signed_travel_speed_mps;
    float target_distance_m;
    float traveled_distance_m;
} FollowTapeAction;

void follow_tape_action_init(FollowTapeAction *action, Drivetrain *drivetrain,
                             TapeSensor *front_sensor,
                             TapeSensor *back_sensor,
                             TapeSensor *left_sensor,
                             TapeFollower *tape_follower);
bool follow_tape_action_start(FollowTapeAction *action,
                              const TaskStepCommand *command,
                              uint32_t now_ms);
TaskActionResult follow_tape_action_update(FollowTapeAction *action,
                                           uint32_t now_ms);
void follow_tape_action_cancel(FollowTapeAction *action);
bool follow_tape_action_report_succeeded(FollowTapeAction *action);
bool follow_tape_action_report_failed(FollowTapeAction *action,
                                      TaskFailure failure);

#ifdef __cplusplus
}
#endif
