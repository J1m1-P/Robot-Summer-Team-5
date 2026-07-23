/* Declares MoveP: closed-loop point-to-point motion with final heading. */
#pragma once

#include "control/drivetrain/endpoint_settle.h"
#include "control/drivetrain/off_tape_motion.h"
#include "control/drivetrain/path_planner.h"
#include "control/drivetrain/speed_profile.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    OffTapeMotionConfig off_tape_motion;
    SpeedProfileConfig speed_profile;
    TapeFollowingControllerConfig heading_controller;
    float distance_tolerance_m;
    float heading_tolerance_rad;
    float max_accel_mps2;
    float max_alpha_rad_s2;
    float max_omega_rad_s;
} MovePConfig;

bool move_p_config_is_valid(const MovePConfig *config);

typedef enum {
    MOVE_P_IDLE = 0,
    MOVE_P_TRANSLATE_AND_TURN,
    MOVE_P_SETTLE_HEADING,
    MOVE_P_COMPLETE,
    MOVE_P_FAULT,
} MovePStatus;

typedef struct {
    const MovePConfig *config;
    PathPlannerLine line;
    OffTapeMotion off_tape_motion;
    TapeFollowingControllerState heading_controller;
    SpeedProfile translation_profile;
    SpeedProfile omega_profile;
    float start_heading_rad;
    float final_heading_rad;
    float heading_delta_rad;
    float max_speed_mps;
    EndpointSettleState settle;
    bool braking_translation;
    MovePStatus status;
} MoveP;

typedef struct {
    DrivetrainBodyVelocity requested_velocity;
    MovePStatus status;
    float distance_to_goal_m;
    float heading_error_rad;
    bool motion_valid;
} MovePOutput;

/* target_x_m/target_y_m and final_heading_rad are world-frame coordinates;
 * heading is CCW-positive. */
esp_err_t move_p_start(MoveP *move, const MovePConfig *config,
                       const MotionEstimate *start,
                       float target_x_m, float target_y_m,
                       float final_heading_rad, float max_speed_mps);

/* `estimate` comes from whichever estimator is active; MoveP never accesses
 * encoder or PMW3610 data directly. */
esp_err_t move_p_update(MoveP *move, const MotionEstimate *estimate,
                        float dt_s, MovePOutput *output);

#ifdef __cplusplus
}
#endif
