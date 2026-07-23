/* Declares MoveC: a source-agnostic closed-loop circular-arc primitive. */
#pragma once

#include "control/drivetrain/endpoint_settle.h"
#include "control/drivetrain/off_tape_motion.h"
#include "control/drivetrain/speed_profile.h"
#include "control/drivetrain/path_planner.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    OffTapeMotionConfig off_tape_motion;
    SpeedProfileConfig speed_profile;
    TapeFollowingControllerConfig heading_controller;
    float arc_length_tolerance_m;
    float radial_tolerance_m;
    float heading_tolerance_rad;
    float max_accel_mps2;
    float max_alpha_rad_s2;
    float max_omega_rad_s;
} MoveCConfig;

bool move_c_config_is_valid(const MoveCConfig *config);

typedef enum {
    MOVE_C_IDLE = 0,
    MOVE_C_RUNNING,
    MOVE_C_COMPLETE,
    MOVE_C_FAULT,
} MoveCStatus;

typedef struct {
    const MoveCConfig *config;
    float center_x_m;
    float center_y_m;
    float radius_m;
    float signed_arc_angle_rad;
    float arc_length_m;
    float start_phase_rad;
    float last_phase_rad;
    float progress_m;
    float max_speed_mps;
    float start_heading_rad;
    float final_heading_rad;
    float last_heading_rad;
    bool initialized_phase;
    bool braking;
    bool settling;
    EndpointSettleState settle;
    MoveCStatus status;
    OffTapeMotion off_tape_motion;
    SpeedProfile translation_profile;
    SpeedProfile omega_profile;
    TapeFollowingControllerState heading_controller;
} MoveC;

typedef struct {
    DrivetrainBodyVelocity requested_velocity;
    MoveCStatus status;
    float remaining_arc_m;
    float radial_error_m;
    float heading_error_rad;
    bool motion_valid;
} MoveCOutput;

/* radius is positive; signed arc angle selects turn direction in the
 * CCW-positive world heading convention. The robot's initial heading is
 * treated as the tangent direction at the start of the arc, so the natural
 * terminal heading is start_heading + arc_angle. */
esp_err_t move_c_start(MoveC *move, const MoveCConfig *config,
                       const MotionEstimate *start, float radius_m,
                       float arc_angle_rad, float max_speed_mps);

/* `estimate` is supplied by the active pose estimator; MoveC never reads a
 * particular encoder or optical-flow implementation directly. */
esp_err_t move_c_update(MoveC *move, const MotionEstimate *estimate,
                        float dt_s, MoveCOutput *output);

#ifdef __cplusplus
}
#endif
