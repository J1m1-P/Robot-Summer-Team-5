/* Implements tape-following session lifecycle and end correction. */
#include "control/tape_following/tape_following_session.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static TapeFollowerSensor locating_sensor(TapeFollowerDirection direction,
                                          TapeLocatingSide side)
{
    /* The available geometry is: px-ccw => +y side, mx-cw => +y side,
     * py-ccw => -x back. The caller explicitly selects cw/ccw. */
    if (direction == TAPE_FOLLOWER_PX && side == TAPE_LOCATING_CCW) {
        return TAPE_FOLLOWER_SIDE;
    }
    if (direction == TAPE_FOLLOWER_MX && side == TAPE_LOCATING_CW) {
        return TAPE_FOLLOWER_SIDE;
    }
    if (direction == TAPE_FOLLOWER_PY && side == TAPE_LOCATING_CCW) {
        return TAPE_FOLLOWER_BACK;
    }
    return TAPE_FOLLOWER_SENSOR_COUNT;
}

static void zero_output(TapeFollowingSessionOutput *output)
{
    memset(output, 0, sizeof(*output));
}

static void direction_velocity(TapeFollowerDirection direction,
                               float speed_mps,
                               DrivetrainBodyVelocity *velocity)
{
    if (direction == TAPE_FOLLOWER_PX) velocity->vx = speed_mps;
    else if (direction == TAPE_FOLLOWER_MX) velocity->vx = -speed_mps;
    else velocity->vy = speed_mps;
}

static bool homing_is_aligned(TapeFollowingSession *session,
                              const TapeFollowingSessionInput *input)
{
    const TapeLineEstimatorConfig *front_config =
        session->follower.config->estimators[TAPE_FOLLOWER_FRONT];
    const TapeLineEstimatorConfig *back_config =
        session->follower.config->estimators[TAPE_FOLLOWER_BACK];
    float front_error = 0.0f;
    float back_error = 0.0f;
    if (front_config == NULL || back_config == NULL ||
        input->sensors[TAPE_FOLLOWER_FRONT] == NULL ||
        input->sensors[TAPE_FOLLOWER_BACK] == NULL ||
        !tape_line_estimator_compute_error(
            input->sensors[TAPE_FOLLOWER_FRONT], front_config,
            &session->follower.estimator_states[TAPE_FOLLOWER_FRONT],
            &front_error) ||
        !tape_line_estimator_compute_error(
            input->sensors[TAPE_FOLLOWER_BACK], back_config,
            &session->follower.estimator_states[TAPE_FOLLOWER_BACK],
            &back_error)) {
        return false;
    }
    return fabsf(front_error) <= session->config->homing_alignment_tolerance &&
           fabsf(back_error) <= session->config->homing_alignment_tolerance;
}

bool tape_following_session_config_is_valid(
    const TapeFollowingSessionConfig *config)
{
    return config != NULL &&
           config->direction < TAPE_FOLLOWER_DIRECTION_COUNT &&
           config->locating_side < TAPE_LOCATING_SIDE_COUNT &&
           isfinite(config->travel_velocity_mps) &&
           config->travel_velocity_mps > 0.0f &&
           isfinite(config->timeout_s) && config->timeout_s > 0.0f &&
           isfinite(config->stop_settle_time_s) &&
           config->stop_settle_time_s >= 0.0f &&
           isfinite(config->correction_speed_mps) &&
           config->correction_speed_mps > 0.0f &&
           isfinite(config->correction_tolerance_m) &&
           config->correction_tolerance_m >= 0.0f &&
           isfinite(config->correction_max_distance_m) &&
           config->correction_max_distance_m > 0.0f &&
           (!config->home_before_following ||
            (config->direction != TAPE_FOLLOWER_PY &&
             isfinite(config->homing_timeout_s) &&
             config->homing_timeout_s > 0.0f &&
             isfinite(config->homing_wiggle_speed_mps) &&
             config->homing_wiggle_speed_mps > 0.0f &&
             isfinite(config->homing_half_period_s) &&
             config->homing_half_period_s > 0.0f &&
             isfinite(config->homing_alignment_tolerance) &&
             config->homing_alignment_tolerance >= 0.0f)) &&
           (!config->stop_at_distance ||
            (isfinite(config->distance_m) && config->distance_m > 0.0f)) &&
           (!config->stop_at_locating_event ||
            locating_sensor(config->direction, config->locating_side) <
                TAPE_FOLLOWER_SENSOR_COUNT) &&
           tape_locating_detector_config_is_valid(&config->locating_detector);
}

esp_err_t tape_following_session_init(
    TapeFollowingSession *session,
    const TapeFollowingSessionConfig *config,
    const TapeFollowerConfig *follower_config)
{
    if (session == NULL || !tape_following_session_config_is_valid(config) ||
        follower_config == NULL) return ESP_ERR_INVALID_ARG;
    if (session->config != NULL) return ESP_ERR_INVALID_STATE;

    memset(session, 0, sizeof(*session));
    esp_err_t error = tape_follower_init(&session->follower, follower_config);
    if (error != ESP_OK) return error;
    error = tape_locating_detector_init(
        &session->locating_detector, &config->locating_detector);
    if (error != ESP_OK) return error;
    session->config = config;
    session->status = TAPE_SESSION_IDLE;
    return ESP_OK;
}

esp_err_t tape_following_session_start(TapeFollowingSession *session)
{
    if (session == NULL) return ESP_ERR_INVALID_ARG;
    if (session->config == NULL) return ESP_ERR_INVALID_STATE;
    if (session->started) return ESP_ERR_INVALID_STATE;

    esp_err_t error = tape_follower_reset(&session->follower);
    if (error != ESP_OK) return error;
    error = tape_locating_detector_reset(&session->locating_detector);
    if (error != ESP_OK) return error;
    session->status = session->config->home_before_following ?
        TAPE_SESSION_HOMING : TAPE_SESSION_FOLLOWING;
    session->locating_marker = TAPE_LOCATING_MARKER_NONE;
    session->elapsed_s = 0.0f;
    session->progress_m = 0.0f;
    session->stop_elapsed_s = 0.0f;
    session->homing_elapsed_s = 0.0f;
    session->homing_phase_s = 0.0f;
    session->homing_sign = 1.0f;
    session->started = true;
    return ESP_OK;
}

esp_err_t tape_following_session_reset(TapeFollowingSession *session)
{
    if (session == NULL) return ESP_ERR_INVALID_ARG;
    if (session->config == NULL) return ESP_ERR_INVALID_STATE;
    session->started = false;
    session->status = TAPE_SESSION_IDLE;
    session->elapsed_s = 0.0f;
    session->progress_m = 0.0f;
    session->stop_elapsed_s = 0.0f;
    session->locating_marker = TAPE_LOCATING_MARKER_NONE;
    session->homing_elapsed_s = 0.0f;
    session->homing_phase_s = 0.0f;
    session->homing_sign = 1.0f;
    return tape_follower_reset(&session->follower);
}

static void finish_stop(TapeFollowingSession *session)
{
    if (session->locating_marker != TAPE_LOCATING_MARKER_NONE) {
        session->status = TAPE_SESSION_END_CORRECTION;
        session->correction_start_progress_m =
            session->progress_m;
        return;
    }
    session->status = TAPE_SESSION_COMPLETE_DISTANCE;
}

esp_err_t tape_following_session_update(
    TapeFollowingSession *session,
    const TapeFollowingSessionInput *input,
    TapeFollowingSessionOutput *output)
{
    if (session == NULL || input == NULL || output == NULL ||
        !isfinite(input->along_tape_delta_m) ||
        !isfinite(input->dt_s) || input->dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (session->config == NULL || !session->started) {
        return ESP_ERR_INVALID_STATE;
    }

    zero_output(output);
    session->elapsed_s += input->dt_s;

    if (session->status == TAPE_SESSION_HOMING) {
        session->homing_elapsed_s += input->dt_s;
        session->homing_phase_s += input->dt_s;
        if (homing_is_aligned(session, input)) {
            /* The homing pose is the odometry origin for this session. */
            session->status = TAPE_SESSION_FOLLOWING;
            session->progress_m = 0.0f;
            session->homing_elapsed_s = 0.0f;
            session->homing_phase_s = 0.0f;
            output->status = session->status;
            output->along_tape_distance_m = 0.0f;
            output->locating_marker = session->locating_marker;
            return ESP_OK;
        }
        if (session->homing_elapsed_s >= session->config->homing_timeout_s) {
            session->status = TAPE_SESSION_FAULT;
        } else {
            if (session->homing_phase_s >=
                session->config->homing_half_period_s) {
                session->homing_phase_s = 0.0f;
                session->homing_sign = -session->homing_sign;
            }
            output->requested_velocity.vy =
                session->homing_sign *
                session->config->homing_wiggle_speed_mps;
            output->status = session->status;
            output->along_tape_distance_m = 0.0f;
            output->motion_valid = true;
            output->locating_marker = session->locating_marker;
            return ESP_OK;
        }
    }

    if (session->status != TAPE_SESSION_FOLLOWING) {
        session->progress_m += input->along_tape_delta_m;
    }
    if (session->elapsed_s >= session->config->timeout_s &&
        session->status == TAPE_SESSION_FOLLOWING) {
        session->status = TAPE_SESSION_TIMEOUT;
    }

    if (session->status == TAPE_SESSION_FOLLOWING) {
        TapeFollowerInput follower_input = {};
        for (int sensor = 0; sensor < TAPE_FOLLOWER_SENSOR_COUNT; ++sensor) {
            follower_input.sensors[sensor] = input->sensors[sensor];
        }
        follower_input.direction = session->config->direction;
        follower_input.travel_velocity_mps =
            session->config->travel_velocity_mps;
        follower_input.along_tape_delta_m = input->along_tape_delta_m;

        TapeFollowerOutput follower_output = {};
        esp_err_t error = tape_follower_update(
            &session->follower, &follower_input, input->dt_s,
            &follower_output);
        output->requested_velocity = follower_output.requested_velocity;
        output->along_tape_distance_m =
            follower_output.along_tape_distance_m;
        session->progress_m = follower_output.along_tape_distance_m;
        if (error != ESP_OK) {
            session->status = TAPE_SESSION_FAULT;
        } else if (!follower_output.motion_valid ||
                   follower_output.status == TAPE_FOLLOWER_LOST) {
            session->status = TAPE_SESSION_TAPE_LOST;
        }

        const TapeFollowerSensor sensor = locating_sensor(
            session->config->direction, session->config->locating_side);
        if (session->status == TAPE_SESSION_FOLLOWING &&
            session->config->stop_at_locating_event &&
            sensor < TAPE_FOLLOWER_SENSOR_COUNT) {
            TapeLocatingDetectionOutput locating_output = {};
            error = tape_locating_detector_update(
                &session->locating_detector, input->sensors[sensor],
                input->along_tape_delta_m, &locating_output);
            if (error != ESP_OK) {
                session->status = TAPE_SESSION_FAULT;
            } else if (locating_output.event) {
                session->locating_marker = locating_output.marker;
                session->correction_target_progress_m =
                    locating_output.marker_center_progress_m;
                session->status = TAPE_SESSION_CONTROLLED_STOP;
                session->stop_elapsed_s = 0.0f;
            }
        }

        if (session->status == TAPE_SESSION_FOLLOWING &&
            session->config->stop_at_distance &&
            session->progress_m >=
                session->config->distance_m) {
            session->status = TAPE_SESSION_CONTROLLED_STOP;
            session->stop_elapsed_s = 0.0f;
        }
    }

    if (session->status == TAPE_SESSION_CONTROLLED_STOP) {
        session->stop_elapsed_s += input->dt_s;
        if (session->stop_elapsed_s >= session->config->stop_settle_time_s) {
            finish_stop(session);
        }
    } else if (session->status == TAPE_SESSION_END_CORRECTION) {
        const float error_m = session->correction_target_progress_m -
                              session->progress_m;
        output->correction_error_m = error_m;
        if (fabsf(error_m) <= session->config->correction_tolerance_m) {
            session->status = TAPE_SESSION_COMPLETE_LOCATING;
        } else if (fabsf(session->progress_m -
                        session->correction_start_progress_m) >
                   session->config->correction_max_distance_m) {
            session->status = TAPE_SESSION_FAULT;
        } else {
            direction_velocity(session->config->direction,
                               copysignf(session->config->correction_speed_mps,
                                         error_m),
                               &output->requested_velocity);
            output->motion_valid = true;
        }
    }

    output->status = session->status;
    output->along_tape_distance_m = session->progress_m;
    if (session->status != TAPE_SESSION_FOLLOWING &&
        session->status != TAPE_SESSION_END_CORRECTION) {
        output->requested_velocity.vx = 0.0f;
        output->requested_velocity.vy = 0.0f;
        output->requested_velocity.omega = 0.0f;
        output->motion_valid = false;
    }
    output->locating_marker = session->locating_marker;
    return ESP_OK;
}
