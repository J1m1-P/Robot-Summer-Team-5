/* Declares RotS: an open-loop, dead-reckoned in-place rotation. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#include "control/drivetrain/speed_profile.h"
#include "control/drivetrain/x_drive_kinematics.h"

#ifdef __cplusplus
extern "C" {
#endif

/* RotS is MoveS's rotation counterpart: it relies solely on the
 * wheel-velocity PI (via the drivetrain facade's own kinematics+PI cycle),
 * with no outer heading correction, so it cannot correct drift, only
 * measure it. That's intentional -- it's the vehicle §3's F_ang calibration
 * trials (repeated known-angle in-place rotations) use, which an outer PID
 * would mask. Reuses speed_profile.c directly for the jerk-bounded ramp:
 * that module's math is unit-agnostic, so the same "m/s" fields work as
 * rad/s here without modification.
 *
 * Genuinely open-loop, same reasoning as MoveS: rot_s_update() takes no
 * heading/odometry input at all. "Remaining angle" is tracked by
 * integrating this module's OWN commanded omega over time
 * (`planned_progress_rad`), not by reading real heading back from odometry
 * -- odometry is exactly what §3's calibration is trying to validate, so
 * using it to decide when to decelerate/stop would let real mechanical
 * error quietly change the rotation's own duration instead of showing up as
 * measurable angle error. The caller reads real odometry/a protractor
 * separately, once ROT_S_COMPLETE is reported, to compare against what was
 * commanded -- that comparison is the calibration signal (F_ang).
 *
 * F_ang is not threaded in yet -- until move_calibration.c (§3.2) exists,
 * there is nothing to apply it to. Unlike F_lon/F_lat (baked into the
 * Jacobian: F_lon/F_lat scale the wheel-speed *output* of
 * x_drive_kinematics_body_to_wheel_velocities(), per the paper's eq. 22-23
 * J_c^-1 = F_lon*F_lat*J^-1), F_ang is explicitly NOT part of that matrix --
 * the paper adds it as a separate command-level correction because F_lat
 * alone didn't fully correct heading error (a second pass, not a Jacobian
 * term). From the paper's eq. 21 (theta1 = F_ang * theta2, commanded over
 * actual), it threads in as a scalar on the *commanded target angle*: to
 * achieve a desired rotation theta_want, call
 * rot_s_start(..., angle_rad = theta_want * F_ang, ...), not as a multiplier
 * on omega/angular speed.
 */
typedef struct {
    SpeedProfileConfig speed_profile;

    /* Once the planned remaining angle is within this, the rotation reports
     * complete and commands zero angular velocity instead of continuing to
     * creep. */
    float angle_tolerance_rad;

    /* Angular acceleration ceiling, shared by every RotS call -- a
     * traction/hardware limit like MoveS's max_accel_mps2, not something
     * that varies rotation to rotation, so it lives in config rather than
     * being passed to rot_s_start() each time. Placeholder default until
     * §3's dynamic calibration derives a real slip-avoidance ceiling; not
     * yet calibrated. */
    float max_alpha_rad_s2;
} RotSConfig;

// Rejects configurations that could produce undefined or unsafe behavior.
bool rot_s_config_is_valid(const RotSConfig *config);

typedef enum {
    ROT_S_IDLE = 0,
    ROT_S_RUNNING,
    ROT_S_COMPLETE,
} RotSStatus;

/* Retains the jerk-bounded ramp, this rotation's target, and its own
 * self-integrated progress -- no heading/odometry reference of any kind. */
typedef struct {
    const RotSConfig *config;
    SpeedProfile profile;
    float angle_rad;  // signed target rotation; positive = counterclockwise
    float max_omega_rad_s;
    float planned_progress_rad; // self-integrated: sum of commanded_omega_rad_s * dt_s
    RotSStatus status;
} RotS;

/* Returns a command compatible with drivetrain_set_body_velocity plus
 * progress. Apply requested_velocity only when motion_valid is true. */
typedef struct {
    DrivetrainBodyVelocity requested_velocity;  // vx = vy = 0, omega commanded
    RotSStatus status;
    float remaining_angle_rad;
    bool motion_valid;
} RotSOutput;

/* Validates config and parameters and prepares a running rotation.
 * `angle_rad` is the signed rotation to command (positive = counterclockwise,
 * matching DrivetrainBodyVelocity.omega's convention); its sign is the only
 * source of direction, there is no separate heading parameter. The angular
 * acceleration ceiling comes from `config->max_alpha_rad_s2`, not a
 * parameter here -- see RotSConfig. Zero-initialize the runtime object
 * before its first call: RotS rot = {0}; */
esp_err_t rot_s_start(
    RotS *rot,
    const RotSConfig *config,
    float angle_rad,
    float max_omega_rad_s);

/* Calculates one motion request purely from elapsed time (`dt_s`) and this
 * rotation's own commanded-omega history -- no heading input, by design
 * (see the header comment above). This function never commands the
 * drivetrain directly. */
esp_err_t rot_s_update(
    RotS *rot,
    float dt_s,
    RotSOutput *output);

#ifdef __cplusplus
}
#endif
