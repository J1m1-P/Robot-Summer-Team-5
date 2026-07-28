/* Defines the closed-loop MOTION primitives' production tuning. */
#include "config/drivetrain/move_config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG_TO_RAD(deg) ((float)(deg) * (float)M_PI / 180.0f)

// Shared lateral-correction PID, reused by MoveL/MoveP/MoveC exactly as
// calibration_main.cpp does (move_p_config.off_tape_motion = move_l_config.off_tape_motion).
//
// integral_gain/derivative_gain: untuned starting points, not verified on
// hardware -- these primitives had never run with any I/D term before this.
// Deliberately biased toward critically-damped/no-overshoot, not
// underdamped: precision here means the *final stopped position* is
// accurate and repeatable (EndpointSettleState's pulse/pause phase, and the
// 0.01m/2deg completion tolerances below, both assume a small, monotonically
// shrinking residual error, not one that rings through zero). Overshoot on a
// wheeled base also means reversing to correct it, and reversal is exactly
// where wheel slip happens -- corrupting the odometry the controller reads
// its own error from. Kd = Kp * 0.3s is deliberately a large fraction of Kp
// to damp overshoot out; Ki = Kp/10 stays conservative and secondary (too
// much I re-introduces overshoot via windup/phase lag, working against the
// same goal). Re-verify via the calibration harness
// (docs/TUNING_ROADMAP.md's movel/movep/arc bring-up) before trusting these:
// if it still overshoots, raise Kd further; if it's sluggish with zero
// overshoot margin to spare, ease it back down.
static const OffTapeMotionConfig LATERAL_MOTION_CONFIG = {
    .controller = {.proportional_gain = 1.0f,
                  .integral_gain = 0.1f,
                  .derivative_gain = 0.3f,
                  .integral_limit = 0.2f,
                  .correction_min = -0.25f,
                  .correction_max = 0.25f},
    .controller_dt_max_s = 0.05f,
};

// Shared heading-correction PID, reused by MoveP/MoveC. Same
// critically-damped-by-design, untuned-starting-point caveat as
// LATERAL_MOTION_CONFIG above.
static const BoundedPidConfig TRANSLATE_HEADING_CONTROLLER_CONFIG = {
    .proportional_gain = 1.0f,
    .integral_gain = 0.1f,
    .derivative_gain = 0.3f,
    .integral_limit = 0.5f,
    .correction_min = -1.0f,
    .correction_max = 1.0f,
};

const MoveLConfig MOVE_L_CONFIG = {
    .off_tape_motion = LATERAL_MOTION_CONFIG,
    .speed_profile = {.max_jerk_mps3 = 10.0f},
    .distance_tolerance_m = 0.01f,
    .max_accel_mps2 = 2.5f,
};

const MovePConfig MOVE_P_CONFIG = {
    .off_tape_motion = LATERAL_MOTION_CONFIG,
    .speed_profile = {.max_jerk_mps3 = 10.0f},
    .heading_controller = TRANSLATE_HEADING_CONTROLLER_CONFIG,
    .distance_tolerance_m = 0.01f,
    .heading_tolerance_rad = DEG_TO_RAD(2.0f),
    .max_accel_mps2 = 2.5f,
    .max_alpha_rad_s2 = 1.5f,
    .max_omega_rad_s = 1.0f,
};

const MoveRConfig MOVE_R_CONFIG = {
    // Same critically-damped-by-design, untuned-starting-point caveat as
    // LATERAL_MOTION_CONFIG above: Ki = Kp/10, Kd = Kp * 0.3s (kept
    // proportionally consistent with the other three controllers rather
    // than the previous unrelated 0.05 carried over from an earlier pass).
    .heading_controller = {.proportional_gain = 2.0f,
                           .integral_gain = 0.2f,
                           .derivative_gain = 0.6f,
                           .integral_limit = 0.5f,
                           .correction_min = -1.0f,
                           .correction_max = 1.0f},
    .speed_profile = {.max_jerk_mps3 = 10.0f},
    .heading_tolerance_rad = DEG_TO_RAD(2.0f),
    .max_alpha_rad_s2 = 1.5f,
    .max_omega_rad_s = 1.0f,
    .controller_dt_max_s = 0.05f,
};

const MoveCConfig MOVE_C_CONFIG = {
    .off_tape_motion = LATERAL_MOTION_CONFIG,
    .speed_profile = {.max_jerk_mps3 = 10.0f},
    .heading_controller = TRANSLATE_HEADING_CONTROLLER_CONFIG,
    .arc_length_tolerance_m = 0.01f,
    .radial_tolerance_m = 0.01f,
    .heading_tolerance_rad = DEG_TO_RAD(2.0f),
    .max_accel_mps2 = 2.5f,
    .max_alpha_rad_s2 = 1.5f,
    .max_omega_rad_s = 1.0f,
};
