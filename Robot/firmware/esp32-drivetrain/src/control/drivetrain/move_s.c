/* Implements MoveS: an open-loop, dead-reckoned straight-line move. */
#include "control/drivetrain/move_s.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include <robot_common/math_utils.h>

/* Below this commanded speed, the jerk-bounded ramp is considered to have
 * actually reached zero rather than just asymptotically approaching it. */
static const float kStoppedSpeedMps = 0.005f;

bool move_s_config_is_valid(const MoveSConfig *config)
{
    return config != NULL &&
           speed_profile_config_is_valid(&config->speed_profile) &&
           isfinite(config->distance_tolerance_m) &&
           config->distance_tolerance_m >= 0.0f &&
           isfinite(config->max_accel_mps2) && config->max_accel_mps2 > 0.0f;
}

esp_err_t move_s_start(
    MoveS *move,
    const MoveSConfig *config,
    float distance_m,
    float heading_rad,
    float max_speed_mps)
{
    if (move == NULL || !move_s_config_is_valid(config) ||
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
    move->status = MOVE_S_RUNNING;
    return ESP_OK;
}

esp_err_t move_s_update(
    MoveS *move,
    float dt_s,
    MoveSOutput *output)
{
    if (move == NULL || output == NULL ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (move->config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(output, 0, sizeof(*output));

    /* Remaining distance against this move's OWN self-integrated progress --
     * never against real position. See move_s.h's header comment: reading
     * real odometry here would let genuine mechanical error quietly change
     * how long the move runs instead of showing up as measurable endpoint
     * error, defeating the calibration this primitive exists to feed. */
    const float remaining_m = move->distance_m - move->planned_progress_m;
    output->remaining_distance_m = remaining_m;

    /* Calibration moves are strictly one-way: once braking starts, zero is
     * the only subsequent target.  Predict the stop from the *current*
     * jerk/acceleration state instead of using v^2/(2a), which assumes an
     * instantaneous acceleration reversal and can start braking too late. */
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
    const esp_err_t error = speed_profile_update(
        &move->profile, &move->config->speed_profile, target_speed_mps,
        move->config->max_accel_mps2, dt_s, &commanded_speed_mps);
    if (error != ESP_OK) {
        return error;
    }

    /* Defense in depth: speed_profile_update()'s overshoot-clamp only
     * guarantees its output won't cross THIS cycle's target -- it is not an
     * absolute ceiling. If target_speed_mps itself is changing cycle to
     * cycle (as it does right where deceleration begins) faster than that
     * relative check can track, nothing else stops commanded_speed_mps from
     * drifting past max_speed_mps (see the equivalent, confirmed-on-hardware
     * issue in rot_s.c). Clamp explicitly (never negative -- MoveS's target
     * is always >= 0, see the sqrt() above), and write the clamped value
     * back into the profile's own state so a bad step can't leave phantom
     * "excess" speed baked into next cycle's calculation. */
    commanded_speed_mps = clamp(commanded_speed_mps, 0.0f, move->max_speed_mps);
    move->profile.commanded_speed_mps = commanded_speed_mps;

    move->planned_progress_m += commanded_speed_mps * dt_s;

    output->requested_velocity.vx = commanded_speed_mps * move->body_direction_x;
    output->requested_velocity.vy = commanded_speed_mps * move->body_direction_y;
    output->requested_velocity.omega = 0.0f;

    if (move->braking && fabsf(commanded_speed_mps) <= kStoppedSpeedMps) {
        move->status = MOVE_S_COMPLETE;
        /* Completion is a zero-command contract, not merely an advisory
         * status.  The final profile sample can be small-but-nonzero; do not
         * pass that residual through to a caller that has already been told
         * the movement is over. */
        output->requested_velocity.vx = 0.0f;
        output->requested_velocity.vy = 0.0f;
        output->requested_velocity.omega = 0.0f;
    }
    output->status = move->status;
    output->motion_valid = true;
    return ESP_OK;
}
