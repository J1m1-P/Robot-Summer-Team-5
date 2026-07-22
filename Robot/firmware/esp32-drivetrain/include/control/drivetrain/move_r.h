/* Declares MoveR: a closed-loop in-place rotation primitive. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#include "control/drivetrain/x_drive_kinematics.h"
#include "control/drivetrain/motion_estimate_adapter.h"
#include "control/drivetrain/speed_profile.h"
#include "control/tape_following/tape_following_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TapeFollowingControllerConfig heading_controller;
    SpeedProfileConfig speed_profile;
    float heading_tolerance_rad;
    float max_alpha_rad_s2;
    float max_omega_rad_s;
} MoveRConfig;

typedef enum {
    MOVE_R_IDLE = 0,
    MOVE_R_RUNNING,
    MOVE_R_COMPLETE,
    MOVE_R_FAULT,
} MoveRStatus;

typedef struct {
    const MoveRConfig *config;
    TapeFollowingControllerState heading_state;
    SpeedProfile profile;
    float target_heading_rad;
    MoveRStatus status;
} MoveR;

typedef struct {
    DrivetrainBodyVelocity requested_velocity;
    MoveRStatus status;
    float heading_error_rad;
    bool motion_valid;
} MoveROutput;

bool move_r_config_is_valid(const MoveRConfig *config);

/* target_heading_rad is an absolute world-frame heading. */
esp_err_t move_r_start(
    MoveR *move,
    const MoveRConfig *config,
    const MotionEstimate *start,
    float target_heading_rad);

esp_err_t move_r_update(
    MoveR *move,
    const MotionEstimate *estimate,
    float dt_s,
    MoveROutput *output);

#ifdef __cplusplus
}
#endif
