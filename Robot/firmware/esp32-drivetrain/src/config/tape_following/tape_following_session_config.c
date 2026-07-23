/* Defines the tape-following session's production tuning defaults. */
#include "config/tape_following/tape_following_session_config.h"

const TapeFollowingSessionConfig TAPE_FOLLOWING_SESSION_CONFIG = {
    // Overridden per-task by tape_session_action.c; placeholders here just
    // need to satisfy tape_following_session_config_is_valid().
    .direction = TAPE_FOLLOWER_PX,
    .locating_side = TAPE_LOCATING_CW,
    .travel_velocity_mps = 0.10f,
    .stop_at_distance = true,
    .distance_m = 1.0f,

    .stop_at_locating_event = false,
    .timeout_s = 20.0f,
    .stop_settle_time_s = 0.10f,
    .correction_speed_mps = 0.08f,
    .correction_tolerance_m = 0.010f,
    .correction_max_distance_m = 0.20f,

    .home_before_following = false,
    .homing_timeout_s = 5.0f,
    .homing_wiggle_speed_mps = 0.05f,
    .homing_half_period_s = 0.25f,
    .homing_alignment_tolerance = 0.05f,

    .locating_detector = {
        .locating_side = TAPE_LOCATING_CW,
        .expected_marker = TAPE_LOCATING_MARKER_SINGLE,
        .minimum_active_channels = 1,
        .confirmation_samples = 1,
        .release_samples = 1,
        .tape_width_m = 0.01905f,
        .double_center_distance_m = 0.045f,
        .spacing_tolerance_m = 0.005f,
    },
};
