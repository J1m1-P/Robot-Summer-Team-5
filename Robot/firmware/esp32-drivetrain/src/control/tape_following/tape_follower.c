/* Implements directional tape following and lost-line recovery. */
#include "control/tape_following/tape_follower.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include <robot_common/math_utils.h>

/* Rejects configurations that could produce undefined or unsafe behavior. */
static bool tape_follower_config_is_valid(const TapeFollowerConfig *config)
{
    if (config == NULL) {
        return false;
    }

    for (int sensor = 0; sensor < TAPE_FOLLOWER_SENSOR_COUNT; sensor++) {
        if (!tape_line_estimator_config_is_valid(config->estimators[sensor])) {
            return false;
        }
    }

    return off_tape_motion_config_is_valid(&config->lateral_motion) &&
           tape_following_kinematics_config_is_valid(&config->heading) &&
           isfinite(config->search.angular_velocity_rad_s) &&
           config->search.angular_velocity_rad_s >= 0.0f &&
           isfinite(config->search.timeout_s) &&
           config->search.timeout_s >= 0.0f &&
           isfinite(config->max_lateral_accel_mps2) &&
           config->max_lateral_accel_mps2 > 0.0f;
}

/* Clears control outputs that must not cross idle, direction, or tape-loss
 * transitions. Estimator history is intentionally preserved for searching. */
static void reset_steering(TapeFollower *follower)
{
    off_tape_motion_reset(&follower->lateral_motion);
    follower->requested_omega_rad_s = 0.0f;
    follower->requested_lateral_velocity_mps = 0.0f;
}

static TapeFollowerSensor sensor_for_direction(TapeFollowerDirection direction)
{
    switch (direction) {
    case TAPE_FOLLOWER_PX: return TAPE_FOLLOWER_FRONT;
    case TAPE_FOLLOWER_MX: return TAPE_FOLLOWER_BACK;
    case TAPE_FOLLOWER_PY: return TAPE_FOLLOWER_SIDE;
    default: return TAPE_FOLLOWER_SENSOR_COUNT;
    }
}

static float search_direction_sign(TapeFollowerDirection direction)
{
    return direction == TAPE_FOLLOWER_MX ? -1.0f : 1.0f;
}

static esp_err_t update_tracking(TapeFollower *follower,
                                 const TapeFollowerInput *input,
    TapeFollowerSensor sensor,
    TapeFollowerDirection direction,
                                 float dt_s,
                                 TapeFollowerOutput *output)
{
    /* Cap controller dt so a scheduling stall cannot create a large integral
     * or derivative impulse on the next valid measurement. */
    OffTapeMotionInput motion_input = {
        .error = output->line_error,
        .travel_velocity_mps = input->travel_velocity_mps,
    };
    OffTapeMotionOutput motion_output = {};
    esp_err_t error = off_tape_motion_update(
        &follower->lateral_motion, &motion_input, dt_s, &motion_output);
    if (error != ESP_OK) return error;

    /* Rate-limit the commanded lateral velocity itself rather than adding a
     * derivative term on line_error -- line_error is a coarse, quantized
     * signal (see tape_line_estimator.c), so differentiating it would react
     * to channel-boundary steps as spikes instead of damping real
     * oscillation. This bounds the output's rate of change directly, which
     * also implicitly smooths omega below since it's derived from this same
     * value. */
    const float max_lateral_change_mps =
        follower->config->max_lateral_accel_mps2 * dt_s;
    const float lateral_velocity_mps = clamp(
        motion_output.requested_velocity.vy,
        follower->requested_lateral_velocity_mps - max_lateral_change_mps,
        follower->requested_lateral_velocity_mps + max_lateral_change_mps);
    follower->requested_lateral_velocity_mps = lateral_velocity_mps;

    switch (direction) {
    case TAPE_FOLLOWER_PX:
        output->requested_velocity.vx = input->travel_velocity_mps;
        output->requested_velocity.vy = lateral_velocity_mps;
        break;
    case TAPE_FOLLOWER_MX:
        output->requested_velocity.vx = -input->travel_velocity_mps;
        output->requested_velocity.vy = lateral_velocity_mps;
        break;
    case TAPE_FOLLOWER_PY:
        output->requested_velocity.vx = -lateral_velocity_mps;
        output->requested_velocity.vy = input->travel_velocity_mps;
        break;
    default:
        return ESP_ERR_INVALID_ARG;
    }

    float controller_dt_s = dt_s;
    if (controller_dt_s > follower->config->lateral_motion.controller_dt_max_s) {
        controller_dt_s = follower->config->lateral_motion.controller_dt_max_s;
    }
    error = tape_following_kinematics_velocity_to_angular_velocity(
        &follower->config->heading,
        direction == TAPE_FOLLOWER_MX
            ? -input->travel_velocity_mps
            : input->travel_velocity_mps,
        lateral_velocity_mps,
        follower->requested_omega_rad_s,
        controller_dt_s,
        &output->requested_velocity.omega);
    if (error != ESP_OK) return error;

    follower->requested_omega_rad_s = output->requested_velocity.omega;
    follower->lost_elapsed_s = 0.0f;
    follower->ever_tracked[sensor] = true;
    output->status = TAPE_FOLLOWER_TRACKING;
    output->motion_valid = true;
    return ESP_OK;
}

static void update_missing_line(TapeFollower *follower,
                                TapeLineEstimatorState *estimator_state,
                                TapeFollowerSensor sensor,
                                float dt_s,
                                TapeFollowerOutput *output)
{
    reset_steering(follower);
    follower->lost_elapsed_s += dt_s;

    if (!follower->ever_tracked[sensor] ||
        follower->lost_elapsed_s >= follower->config->search.timeout_s) {
        output->status = TAPE_FOLLOWER_LOST;
        return;
    }

    float search_direction = 0.0f;
    if (estimator_state->last_known_error > 0.0f) {
        search_direction = 1.0f;
    } else if (estimator_state->last_known_error < 0.0f) {
        search_direction = -1.0f;
    }

    /* Turn the leading end toward the last-known tape side. Forward travel
     * uses the front sensor and reverse travel mirrors the turn for the back. */
    output->requested_velocity.omega =
        -search_direction_sign(follower->active_direction) * search_direction *
        follower->config->search.angular_velocity_rad_s;
    output->status = TAPE_FOLLOWER_SEARCHING;
    output->motion_valid = search_direction != 0.0f &&
                           follower->config->search.angular_velocity_rad_s > 0.0f;
}

esp_err_t tape_follower_init(TapeFollower *follower,
                             const TapeFollowerConfig *config)
{
    if (follower == NULL || !tape_follower_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (follower->config != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(follower, 0, sizeof(*follower));
    follower->config = config;
    return off_tape_motion_init(&follower->lateral_motion,
                                &config->lateral_motion);
}

esp_err_t tape_follower_reset(TapeFollower *follower)
{
    if (follower == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (follower->config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    for (int sensor = 0; sensor < TAPE_FOLLOWER_SENSOR_COUNT; sensor++) {
        tape_line_estimator_reset(&follower->estimator_states[sensor]);
        follower->ever_tracked[sensor] = false;
    }
    off_tape_motion_reset(&follower->lateral_motion);

    follower->lost_elapsed_s = 0.0f;
    follower->requested_omega_rad_s = 0.0f;
    follower->requested_lateral_velocity_mps = 0.0f;
    follower->active_direction = TAPE_FOLLOWER_DIRECTION_COUNT;
    follower->along_tape_distance_m = 0.0f;
    return ESP_OK;
}

esp_err_t tape_follower_update(TapeFollower *follower,
                               const TapeFollowerInput *input,
                               float dt_s,
                               TapeFollowerOutput *output)
{
    if (follower == NULL || input == NULL || output == NULL ||
        !isfinite(input->travel_velocity_mps) ||
        input->travel_velocity_mps < 0.0f ||
        !isfinite(input->along_tape_delta_m) ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (follower->config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(output, 0, sizeof(*output));
    /* A zero travel request idles the behavior instead of choosing a sensor or
     * refreshing the drivetrain watchdog with a zero motion command. */
    if (input->direction >= TAPE_FOLLOWER_DIRECTION_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    if (input->travel_velocity_mps == 0.0f) {
        if (follower->active_direction != TAPE_FOLLOWER_DIRECTION_COUNT) {
            reset_steering(follower);
        }
        follower->active_direction = TAPE_FOLLOWER_DIRECTION_COUNT;
        follower->lost_elapsed_s = 0.0f;
        output->status = TAPE_FOLLOWER_IDLE;
        output->along_tape_distance_m = follower->along_tape_distance_m;
        return ESP_OK;
    }
    const TapeFollowerSensor sensor = sensor_for_direction(input->direction);
    if (sensor >= TAPE_FOLLOWER_SENSOR_COUNT || input->sensors[sensor] == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Do not carry derivative or integral history across a direction change. */
    if (input->direction != follower->active_direction) {
        reset_steering(follower);
        follower->lost_elapsed_s = 0.0f;
        follower->active_direction = input->direction;
    }

    /* Integrate signed net progress, rather than commanded speed or absolute
     * displacement. Steering oscillations can create lateral path length, but
     * they must not inflate progress along the tape. */
    follower->along_tape_distance_m += input->along_tape_delta_m;
    TapeLineEstimatorState *estimator_state =
        &follower->estimator_states[sensor];
    const bool line_present = tape_line_estimator_compute_error(
        input->sensors[sensor], follower->config->estimators[sensor],
        estimator_state,
        &output->line_error);

    if (!line_present) {
        /* Search only after this directional sensor has acquired tape. This
         * prevents front-sensor history from affecting backward recovery. */
        update_missing_line(follower, estimator_state, sensor, dt_s, output);
        output->along_tape_distance_m = follower->along_tape_distance_m;
        return ESP_OK;
    }

    esp_err_t error = update_tracking(
        follower, input, sensor, input->direction, dt_s, output);
    output->along_tape_distance_m = follower->along_tape_distance_m;
    return error;
}
