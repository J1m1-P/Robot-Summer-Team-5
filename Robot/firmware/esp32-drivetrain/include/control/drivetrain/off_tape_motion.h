/* Declares the shared off-tape (open-field) motion PID behavior. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#include "control/drivetrain/x_drive_kinematics.h"
#include "control/tape_following/tape_following_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Combines the shared PID gains with a bounded control-loop period.
 *
 * This is deliberately the same generic bounded PID used by tape following
 * (control/tape_following/tape_following_controller.h) -- off-tape motion
 * has no line-following-specific behavior of its own, only a different error
 * source (see off_tape_motion_update's `error` input) and no heading
 * coupling (correction is applied laterally; MoveP's terminal heading phase
 * layers rotation on separately, outside this module).
 */
typedef struct {
    TapeFollowingControllerConfig controller;

    /* Upper bound used for PID integration and differentiation after loop stalls. */
    float controller_dt_max_s;
} OffTapeMotionConfig;

/* Supplies the caller-computed path error and the requested along-path speed.
 * The error source (cross-track distance for MoveL, distance-to-point for
 * MoveP, distance-off-arc for MoveC) is computed outside this module so the
 * PID stays agnostic to where the error comes from -- an encoder-odometry
 * source today, potentially a PMW3610 optical-flow source later. */
typedef struct {
    float error;
    float travel_velocity_mps;
} OffTapeMotionInput;

/* Returns a command compatible with drivetrain_set_body_velocity. Apply
 * requested_velocity only when motion_valid is true. */
typedef struct {
    DrivetrainBodyVelocity requested_velocity;
    bool motion_valid;
} OffTapeMotionOutput;

/* Retains PID integral/derivative history for one behavior instance. */
typedef struct {
    const OffTapeMotionConfig *config;
    TapeFollowingControllerState controller_state;
} OffTapeMotion;

// Rejects configurations that could produce undefined or unsafe behavior.
bool off_tape_motion_config_is_valid(const OffTapeMotionConfig *config);

/* Validates the configuration and prepares an idle off-tape motion behavior.
 * Zero-initialize the runtime object before its first init call:
 * OffTapeMotion motion = {0}; */
esp_err_t off_tape_motion_init(OffTapeMotion *motion,
                               const OffTapeMotionConfig *config);

/* Clears PID integral/derivative history without changing config. */
esp_err_t off_tape_motion_reset(OffTapeMotion *motion);

/* Calculates one motion request from the caller-supplied path error.
 * This function never commands the drivetrain directly. */
esp_err_t off_tape_motion_update(OffTapeMotion *motion,
                                 const OffTapeMotionInput *input,
                                 float dt_s,
                                 OffTapeMotionOutput *output);

#ifdef __cplusplus
}
#endif
