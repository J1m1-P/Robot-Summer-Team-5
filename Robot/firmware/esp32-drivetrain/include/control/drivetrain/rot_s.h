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
 * F_ang is not threaded in yet -- until move_calibration.c (§3.2) exists,
 * there is nothing to apply it to. It can be threaded in later as a uniform
 * scalar on the commanded angular speed, the rotational analog of how
 * F_lon applies to MoveS's linear speed.
 */
typedef struct {
    SpeedProfileConfig speed_profile;

    /* Once the remaining angle is within this, the rotation reports
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

/* Retains the jerk-bounded ramp, the starting heading, and this rotation's target. */
typedef struct {
    const RotSConfig *config;
    SpeedProfile profile;
    float start_heading_rad;
    float angle_rad;  // signed target rotation; positive = counterclockwise
    float max_omega_rad_s;
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

/* Validates config and parameters, captures `start_heading_rad` as the
 * reference point progress is measured from, and prepares a running
 * rotation. `angle_rad` is the signed rotation to command (positive =
 * counterclockwise, matching DrivetrainBodyVelocity.omega's convention);
 * its sign is the only source of direction, there is no separate heading
 * parameter. The angular acceleration ceiling comes from
 * `config->max_alpha_rad_s2`, not a parameter here -- see RotSConfig.
 * Zero-initialize the runtime object before its first call:
 * RotS rot = {0}; */
esp_err_t rot_s_start(
    RotS *rot,
    const RotSConfig *config,
    float start_heading_rad,
    float angle_rad,
    float max_omega_rad_s);

/* Calculates one motion request from the caller-supplied current heading
 * (read from odometry each cycle -- DrivetrainPose.heading_rad accumulates
 * continuously rather than wrapping, so a plain subtraction against
 * start_heading_rad gives exact progress with no unwrap logic needed).
 * This function never commands the drivetrain directly. */
esp_err_t rot_s_update(
    RotS *rot,
    float current_heading_rad,
    float dt_s,
    RotSOutput *output);

#ifdef __cplusplus
}
#endif
