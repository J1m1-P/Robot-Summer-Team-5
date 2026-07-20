/* Declares MoveS: an open-loop, dead-reckoned straight-line move. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#include "control/drivetrain/odometry.h"
#include "control/drivetrain/speed_profile.h"
#include "control/drivetrain/x_drive_kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* MoveS relies solely on the wheel-velocity PI (via the drivetrain facade's
 * own kinematics+PI cycle) -- unlike MoveL/MoveP/MoveC, it has no outer
 * position/heading correction, so it cannot correct drift, only measure it.
 * That's intentional: it's also the vehicle §3's calibration trials use to
 * derive F_lon/F_lat/F_ang, which would be masked by an outer PID.
 *
 * F_lon/F_lat/F_ang are not threaded in yet -- until move_calibration.c
 * (§3.2) exists, there is nothing to apply them to. F_lon can be threaded in
 * later as a uniform scalar on the commanded body-speed magnitude (linear
 * with the Jacobian, so this is exact, not an approximation); F_lat needs a
 * per-wheel post-Jacobian correction the drivetrain facade doesn't currently
 * expose a hook for, and will need that resolved in §3.
 */
typedef struct {
    SpeedProfileConfig speed_profile;

    /* Once the remaining distance is within this, the move reports complete
     * and commands zero velocity instead of continuing to creep forward. */
    float distance_tolerance_m;
} MoveSConfig;

// Rejects configurations that could produce undefined or unsafe behavior.
bool move_s_config_is_valid(const MoveSConfig *config);

typedef enum {
    MOVE_S_IDLE = 0,
    MOVE_S_RUNNING,
    MOVE_S_COMPLETE,
} MoveSStatus;

/* Retains the jerk-bounded ramp, the starting pose, and this move's targets. */
typedef struct {
    const MoveSConfig *config;
    SpeedProfile profile;
    DrivetrainPose start_pose;
    float distance_m;
    float world_direction_x;  // unit vector, world frame, captured at start
    float world_direction_y;
    float body_direction_x;   // unit vector, body frame (constant: no rotation)
    float body_direction_y;
    float max_speed_mps;
    float max_accel_mps2;
    MoveSStatus status;
} MoveS;

/* Returns a command compatible with drivetrain_set_body_velocity plus
 * progress. Apply requested_velocity only when motion_valid is true. */
typedef struct {
    DrivetrainBodyVelocity requested_velocity;
    MoveSStatus status;
    float remaining_distance_m;
    bool motion_valid;
} MoveSOutput;

/* Validates config and parameters, captures `start_pose` as the reference
 * point progress is measured from, and prepares a running move. `heading_rad`
 * is the direction of travel in the robot's body frame (0 = forward,
 * positive = toward +vy/strafe-right), not a target orientation -- MoveS
 * never commands rotation. Zero-initialize the runtime object before its
 * first call: MoveS move = {0}; */
esp_err_t move_s_start(
    MoveS *move,
    const MoveSConfig *config,
    const DrivetrainPose *start_pose,
    float distance_m,
    float heading_rad,
    float max_speed_mps,
    float max_accel_mps2);

/* Calculates one motion request from the caller-supplied current pose
 * (read from odometry each cycle). This function never commands the
 * drivetrain directly. */
esp_err_t move_s_update(
    MoveS *move,
    const DrivetrainPose *current_pose,
    float dt_s,
    MoveSOutput *output);

#ifdef __cplusplus
}
#endif
