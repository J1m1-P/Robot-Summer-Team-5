/* Implements MoveL: a source-agnostic closed-loop straight-line primitive. */
#include "control/drivetrain/move_l.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include <robot_common/math_utils.h>

static const float kStoppedSpeedMps = 0.005f;

bool move_l_config_is_valid(const MoveLConfig *config)
{
    return config != NULL &&
           off_tape_motion_config_is_valid(&config->off_tape_motion) &&
           speed_profile_config_is_valid(&config->speed_profile) &&
           isfinite(config->distance_tolerance_m) &&
           config->distance_tolerance_m >= 0.0f &&
           isfinite(config->max_accel_mps2) && config->max_accel_mps2 > 0.0f;
}

esp_err_t move_l_start(
    MoveL *move,
    const MoveLConfig *config,
    float distance_m,
    float heading_rad,
    float max_speed_mps)
{
    if (move == NULL || !move_l_config_is_valid(config) ||
        !isfinite(distance_m) || distance_m <= 0.0f ||
        !isfinite(heading_rad) ||
        !isfinite(max_speed_mps) || max_speed_mps <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(move, 0, sizeof(*move));
    move->config = config;
    move->distance_m = distance_m;
    move->body_direction_x = cosf(heading_rad);
    move->body_direction_y = sinf(heading_rad);
    move->max_speed_mps = max_speed_mps;
    speed_profile_reset(&move->profile, 0.0f);

    const esp_err_t error = off_tape_motion_init(
        &move->off_tape_motion, &config->off_tape_motion);
    if (error != ESP_OK) return error;

    move->status = MOVE_L_RUNNING;
    return ESP_OK;
}

esp_err_t move_l_update(
    MoveL *move,
    const MoveLInput *input,
    float dt_s,
    MoveLOutput *output)
{
    if (output != NULL) memset(output, 0, sizeof(*output));
    if (move == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (move->config == NULL) return ESP_ERR_INVALID_STATE;
    if (input == NULL ||
        !input->valid ||
        !isfinite(input->along_track_progress_m) ||
        !isfinite(input->cross_track_error_m) ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        move->status = MOVE_L_FAULT;
        output->status = MOVE_L_FAULT;
        return ESP_ERR_INVALID_ARG;
    }
    if (move->status == MOVE_L_COMPLETE || move->status == MOVE_L_FAULT) {
        output->status = move->status;
        return move->status == MOVE_L_COMPLETE ? ESP_OK : ESP_ERR_INVALID_STATE;
    }
    if (dt_s > move->config->off_tape_motion.controller_dt_max_s) {
        move->status = MOVE_L_FAULT;
        output->status = MOVE_L_FAULT;
        return ESP_ERR_INVALID_STATE;
    }

    const float remaining_m = move->distance_m - input->along_track_progress_m;
    output->remaining_distance_m = remaining_m;

    /* Use the real, source-agnostic along-track estimate to decide when to
     * brake.  Once braking begins, zero remains the speed target; lateral
     * feedback may continue correcting until the profile has stopped. */
    if (!move->braking) {
        float stopping_distance_m = 0.0f;
        const esp_err_t error = speed_profile_predict_stopping_distance(
            &move->profile, &move->config->speed_profile,
            move->config->max_accel_mps2, dt_s, kStoppedSpeedMps,
            &stopping_distance_m);
        if (error != ESP_OK) return error;
        if (remaining_m <= stopping_distance_m + move->config->distance_tolerance_m) {
            move->braking = true;
        }
    }

    const float target_speed_mps = move->braking ? 0.0f : move->max_speed_mps;
    float commanded_speed_mps = 0.0f;
    const esp_err_t profile_error = speed_profile_update(
        &move->profile, &move->config->speed_profile, target_speed_mps,
        move->config->max_accel_mps2, dt_s, &commanded_speed_mps);
    if (profile_error != ESP_OK) return profile_error;
    commanded_speed_mps = clamp(commanded_speed_mps, 0.0f, move->max_speed_mps);
    move->profile.commanded_speed_mps = commanded_speed_mps;

    OffTapeMotionOutput correction = {};
    const OffTapeMotionInput correction_input = {
        .error = input->cross_track_error_m,
        .travel_velocity_mps = commanded_speed_mps,
    };
    const esp_err_t correction_error = off_tape_motion_update(
        &move->off_tape_motion, &correction_input, dt_s, &correction);
    if (correction_error != ESP_OK) return correction_error;

    /* OffTapeMotion emits path-frame vx/vy.  Rotate both components into the
     * requested body-frame travel direction before returning the command. */
    const float lateral_x = -move->body_direction_y;
    const float lateral_y = move->body_direction_x;
    output->requested_velocity.vx =
        correction.requested_velocity.vx * move->body_direction_x +
        correction.requested_velocity.vy * lateral_x;
    output->requested_velocity.vy =
        correction.requested_velocity.vx * move->body_direction_y +
        correction.requested_velocity.vy * lateral_y;
    output->requested_velocity.omega = 0.0f;

    if (move->braking && fabsf(commanded_speed_mps) <= kStoppedSpeedMps) {
        if (fabsf(remaining_m) > move->config->distance_tolerance_m) {
            /* MoveL has only cross-track correction; it must not pretend it
             * can repair a residual along-track miss after its one-way
             * profile has stopped. */
            move->status = MOVE_L_FAULT;
            output->requested_velocity.vx = 0.0f;
            output->requested_velocity.vy = 0.0f;
        } else if (fabsf(input->cross_track_error_m) <= move->config->distance_tolerance_m) {
            move->status = MOVE_L_COMPLETE;
            output->requested_velocity.vx = 0.0f;
            output->requested_velocity.vy = 0.0f;
            output->requested_velocity.omega = 0.0f;
        }
    }
    output->status = move->status;
    output->motion_valid = move->status != MOVE_L_FAULT;
    return ESP_OK;
}
