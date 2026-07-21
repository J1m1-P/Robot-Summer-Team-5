/* Implements directional tape following and lost-line recovery. */
#include "control/tape_following/tape_follower.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

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

    return tape_following_controller_config_is_valid(&config->controller) &&
           tape_following_kinematics_config_is_valid(&config->heading) &&
           isfinite(config->search.angular_velocity_rad_s) &&
           config->search.angular_velocity_rad_s >= 0.0f &&
           isfinite(config->search.timeout_s) &&
           config->search.timeout_s >= 0.0f &&
           isfinite(config->controller_dt_max_s) &&
           config->controller_dt_max_s > 0.0f;
}

/* Clears control outputs that must not cross idle, direction, or tape-loss
 * transitions. Estimator history is intentionally preserved for searching. */
static void reset_steering(TapeFollower *follower)
{
    tape_following_controller_reset(&follower->controller_state);
    follower->requested_omega_rad_s = 0.0f;
}

static int8_t direction_from_velocity(float travel_velocity_mps)
{
    if (travel_velocity_mps > 0.0f) return 1;
    if (travel_velocity_mps < 0.0f) return -1;
    return 0;
}

static esp_err_t update_tracking(TapeFollower *follower,
                                 const TapeFollowerInput *input,
                                 TapeFollowerSensor sensor,
                                 float dt_s,
                                 TapeFollowerOutput *output)
{
    /* Cap controller dt so a scheduling stall cannot create a large integral
     * or derivative impulse on the next valid measurement. */
    float controller_dt_s = dt_s;
    if (controller_dt_s > follower->config->controller_dt_max_s) {
        controller_dt_s = follower->config->controller_dt_max_s;
        tape_following_controller_reset(&follower->controller_state);
    }

    output->requested_velocity.vx = input->travel_velocity_mps;
    output->requested_velocity.vy = tape_following_controller_update(
        &follower->controller_state, &follower->config->controller,
        output->line_error, controller_dt_s);

    esp_err_t error = tape_following_kinematics_velocity_to_angular_velocity(
        &follower->config->heading,
        output->requested_velocity.vx,
        output->requested_velocity.vy,
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
        -(float)follower->active_direction * search_direction *
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
    return ESP_OK;
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
    tape_following_controller_reset(&follower->controller_state);

    follower->lost_elapsed_s = 0.0f;
    follower->requested_omega_rad_s = 0.0f;
    follower->active_direction = 0;
    return ESP_OK;
}

esp_err_t tape_follower_update(TapeFollower *follower,
                               const TapeFollowerInput *input,
                               float dt_s,
                               TapeFollowerOutput *output)
{
    if (follower == NULL || input == NULL || output == NULL ||
        input->sensors[TAPE_FOLLOWER_FRONT] == NULL ||
        input->sensors[TAPE_FOLLOWER_BACK] == NULL ||
        !isfinite(input->travel_velocity_mps) ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (follower->config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(output, 0, sizeof(*output));
    /* A zero travel request idles the behavior instead of choosing a sensor or
     * refreshing the drivetrain watchdog with a zero motion command. */
    const int8_t requested_direction =
        direction_from_velocity(input->travel_velocity_mps);

    if (requested_direction == 0) { 
        if (follower->active_direction != 0) {
            reset_steering(follower);
        }
        follower->active_direction = 0;
        follower->lost_elapsed_s = 0.0f;
        output->status = TAPE_FOLLOWER_IDLE;
        return ESP_OK;
    }

    /* Do not carry derivative or integral history across a direction change. */
    if (requested_direction != follower->active_direction) {
        reset_steering(follower);
        follower->lost_elapsed_s = 0.0f;
        follower->active_direction = requested_direction;
    }

    const TapeFollowerSensor sensor = requested_direction > 0
                                          ? TAPE_FOLLOWER_FRONT
                                          : TAPE_FOLLOWER_BACK;
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
        return ESP_OK;
    }

    return update_tracking(follower, input, sensor, dt_s, output);
}
