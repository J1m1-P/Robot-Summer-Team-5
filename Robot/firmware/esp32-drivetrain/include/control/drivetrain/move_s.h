/* Declares MoveS: an open-loop, dead-reckoned straight-line move. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

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
 * Genuinely open-loop: move_s_update() takes no pose/odometry input at all.
 * "Remaining distance" is tracked by integrating this module's OWN commanded
 * speed over time (`planned_progress_m`), not by reading real position back
 * from odometry -- odometry is exactly what §3's calibration is trying to
 * validate, so using it to decide when to decelerate/stop would let real
 * mechanical error (wheel slip, radius mismatch) quietly change the move's
 * own duration instead of showing up as measurable endpoint error. The
 * caller is expected to read real odometry/physical measurement themselves,
 * separately, to compare against what was commanded once the move reports
 * MOVE_S_COMPLETE -- that comparison is the calibration signal.
 *
 * `heading_rad` is a body-frame direction: zero is body +x/front and positive
 * angles point toward body +y/left. In a CCW-positive world frame, that is an
 * addition to the current world heading.
 *
 * MoveS deliberately bypasses the advanced-motion calibration path. It is a
 * measurement primitive; applying its own trial factors would invalidate the
 * baseline experiment.
 */
typedef struct {
    SpeedProfileConfig speed_profile;

    /* Once the planned remaining distance is within this, the move reports
     * complete and commands zero velocity instead of continuing to creep
     * forward. */
    float distance_tolerance_m;

    /* Acceleration ceiling, shared by every MoveS call -- this is a
     * traction/hardware limit (how hard the wheels can accelerate before
     * slipping), not something that varies move to move the way max_speed
     * does, so it lives in config rather than being passed to move_s_start()
     * each time. Placeholder default until §3's dynamic calibration derives
     * a real slip-avoidance ceiling; not yet calibrated. */
    float max_accel_mps2;
} MoveSConfig;

// Rejects configurations that could produce undefined or unsafe behavior.
bool move_s_config_is_valid(const MoveSConfig *config);

typedef enum {
    MOVE_S_IDLE = 0,
    MOVE_S_RUNNING,
    MOVE_S_COMPLETE,
} MoveSStatus;

/* Retains the jerk-bounded ramp, this move's target, and its own
 * self-integrated progress -- no pose/odometry reference of any kind. */
typedef struct {
    const MoveSConfig *config;
    SpeedProfile profile;
    float distance_m;
    float body_direction_x;   // unit vector, body frame (constant: no rotation)
    float body_direction_y;
    float max_speed_mps;
    float planned_progress_m; // self-integrated: sum of commanded_speed_mps * dt_s
    bool braking;             // once set, target speed remains zero; never reverse
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

/* Validates config and parameters and prepares a running move. `heading_rad`
 * is the direction of travel in the robot's body frame (0 = forward,
 * positive = toward +vy/strafe-left), not a target orientation -- MoveS
 * never commands rotation. The acceleration ceiling comes from
 * `config->max_accel_mps2`, not a parameter here -- see MoveSConfig.
 * Zero-initialize the runtime object before its first call:
 * MoveS move = {0}; */
esp_err_t move_s_start(
    MoveS *move,
    const MoveSConfig *config,
    float distance_m,
    float heading_rad,
    float max_speed_mps);

/* Calculates one motion request purely from elapsed time (`dt_s`) and this
 * move's own commanded-speed history -- no pose input, by design (see the
 * header comment above). This function never commands the drivetrain
 * directly. */
esp_err_t move_s_update(
    MoveS *move,
    float dt_s,
    MoveSOutput *output);

#ifdef __cplusplus
}
#endif
