/* Declares the tape-following session lifecycle and stop-reason controller. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#include "control/drivetrain/x_drive_kinematics.h"
#include "control/tape_following/tape_follower.h"
#include "sensing/tape_following/tape_locating_detection.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TAPE_SESSION_IDLE = 0,
    TAPE_SESSION_HOMING,
    TAPE_SESSION_FOLLOWING,
    TAPE_SESSION_CONTROLLED_STOP,
    TAPE_SESSION_END_CORRECTION,
    TAPE_SESSION_COMPLETE_DISTANCE,
    TAPE_SESSION_COMPLETE_LOCATING,
    TAPE_SESSION_TIMEOUT,
    TAPE_SESSION_TAPE_LOST,
    TAPE_SESSION_FAULT,
    TAPE_SESSION_STOPPED,
} TapeFollowingSessionStatus;

typedef struct {
    TapeFollowerDirection direction;
    TapeLocatingSide locating_side;
    float travel_velocity_mps;
    bool stop_at_distance;
    float distance_m;
    bool stop_at_locating_event;
    float timeout_s;
    float stop_settle_time_s;
    float correction_speed_mps;
    float correction_tolerance_m;
    float correction_max_distance_m;
    bool home_before_following;
    float homing_timeout_s;
    float homing_wiggle_speed_mps;
    float homing_half_period_s;
    float homing_alignment_tolerance;
    TapeLocatingDetectorConfig locating_detector;
} TapeFollowingSessionConfig;

typedef struct {
    const TapeSensor *sensors[TAPE_FOLLOWER_SENSOR_COUNT];
    float along_tape_delta_m;
    float dt_s;
} TapeFollowingSessionInput;

typedef struct {
    DrivetrainBodyVelocity requested_velocity;
    TapeFollowingSessionStatus status;
    TapeLocatingMarker locating_marker;
    float along_tape_distance_m;
    float correction_error_m;
    bool motion_valid;
} TapeFollowingSessionOutput;

typedef struct {
    const TapeFollowingSessionConfig *config;
    TapeFollower follower;
    TapeLocatingDetector locating_detector;
    TapeFollowingSessionStatus status;
    TapeLocatingMarker locating_marker;
    float elapsed_s;
    float progress_m;
    float stop_elapsed_s;
    float correction_start_progress_m;
    float correction_target_progress_m;
    float homing_elapsed_s;
    float homing_phase_s;
    float homing_sign;
    bool started;
} TapeFollowingSession;

bool tape_following_session_config_is_valid(
    const TapeFollowingSessionConfig *config);

esp_err_t tape_following_session_init(
    TapeFollowingSession *session,
    const TapeFollowingSessionConfig *config,
    const TapeFollowerConfig *follower_config);

esp_err_t tape_following_session_start(TapeFollowingSession *session);
esp_err_t tape_following_session_reset(TapeFollowingSession *session);

esp_err_t tape_following_session_update(
    TapeFollowingSession *session,
    const TapeFollowingSessionInput *input,
    TapeFollowingSessionOutput *output);

#ifdef __cplusplus
}
#endif
