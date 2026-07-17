/* Implements directional tape following and lost-line recovery. */
#include "control/tape_follower.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* Validates one bounded PID configuration before it becomes runtime state. */
static bool controller_config_is_valid(
    const TapeFollowingControllerConfig *config)
{
    return config != NULL &&
           isfinite(config->proportional_gain) &&
           isfinite(config->integral_gain) &&
           isfinite(config->derivative_gain) &&
           isfinite(config->integral_limit) &&
           config->integral_limit >= 0.0f &&
           isfinite(config->correction_min) &&
           isfinite(config->correction_max) &&
           config->correction_min <= config->correction_max;
}

/* Rejects configurations that could produce undefined or unsafe behavior. */
static bool tape_follower_config_is_valid(const TapeFollowerConfig *config)
{
    return config != NULL && config->front_estimator != NULL &&
           config->back_estimator != NULL &&
           controller_config_is_valid(&config->controller) &&
           isfinite(config->search_duty) && config->search_duty >= 0.0f &&
           config->search_duty <= 1.0f && isfinite(config->lost_timeout_s) &&
           config->lost_timeout_s >= 0.0f &&
           isfinite(config->controller_dt_max_s) &&
           config->controller_dt_max_s > 0.0f;
}

/* Resets feedback whenever motion direction or tape availability changes. */
static void reset_controller(TapeFollower *follower)
{
    tape_following_controller_reset(&follower->controller_state);
}

esp_err_t tape_follower_init(TapeFollower *follower,
                             const TapeFollowerConfig *config)
{
    if (follower == NULL || !tape_follower_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (follower->initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(follower, 0, sizeof(*follower));
    follower->config = config;
    follower->status = TAPE_FOLLOWER_IDLE;
    follower->initialized = true;
    return ESP_OK;
}

esp_err_t tape_follower_reset(TapeFollower *follower)
{
    if (follower == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!follower->initialized || follower->config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    tape_line_estimator_reset(&follower->front_estimator_state);
    tape_line_estimator_reset(&follower->back_estimator_state);
    reset_controller(follower);

    follower->lost_elapsed_s = 0.0f;
    follower->status = TAPE_FOLLOWER_IDLE;
    follower->active_direction = 0;
    follower->front_ever_tracked = false;
    follower->back_ever_tracked = false;
    return ESP_OK;
}

esp_err_t tape_follower_update(TapeFollower *follower,
                               const TapeFollowerInput *input,
                               float dt_s,
                               TapeFollowerOutput *output)
{
    if (follower == NULL || input == NULL || output == NULL ||
        input->front_sensor == NULL || input->back_sensor == NULL ||
        !isfinite(input->travel_duty) || fabsf(input->travel_duty) > 1.0f ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!follower->initialized || follower->config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(output, 0, sizeof(*output));
    /* A zero travel request idles the behavior instead of choosing a sensor or
     * refreshing the drivetrain watchdog with a zero motion command. */
    int8_t requested_direction = 0;
    if (input->travel_duty > 0.0f) {
        requested_direction = 1;
    } else if (input->travel_duty < 0.0f) {
        requested_direction = -1;
    }

    if (requested_direction == 0) { 
        if (follower->active_direction != 0) {
            reset_controller(follower);
        }
        follower->active_direction = 0;
        follower->lost_elapsed_s = 0.0f;
        follower->status = TAPE_FOLLOWER_IDLE;
        output->status = follower->status;
        return ESP_OK;
    }

    /* Do not carry derivative or integral history across a direction change. */
    if (requested_direction != follower->active_direction) {
        reset_controller(follower);
        follower->lost_elapsed_s = 0.0f;
        follower->active_direction = requested_direction;
    }

    const bool moving_forward = requested_direction > 0;
    const TapeSensor *active_sensor = moving_forward
                                          ? input->front_sensor
                                          : input->back_sensor;
    const TapeLineEstimatorConfig *active_estimator =
        moving_forward ? follower->config->front_estimator
                       : follower->config->back_estimator;
    TapeLineEstimatorState *active_estimator_state =
        moving_forward ? &follower->front_estimator_state
                       : &follower->back_estimator_state;
    bool *active_ever_tracked = moving_forward
                                    ? &follower->front_ever_tracked
                                    : &follower->back_ever_tracked;

    output->using_front_sensor = moving_forward;
    output->line_present = tape_line_estimator_compute_error(
        active_sensor, active_estimator, active_estimator_state,
        &output->line_error);

    if (output->line_present) {
        /* Cap controller dt so a scheduling stall cannot create a large integral
         * or derivative impulse on the next valid measurement. */
        float controller_dt = dt_s;
        if (controller_dt > follower->config->controller_dt_max_s) {
            controller_dt = follower->config->controller_dt_max_s;
            reset_controller(follower);
        }

        output->requested_motion.x = tape_following_controller_update(
            &follower->controller_state, &follower->config->controller,
            output->line_error, controller_dt);
        output->requested_motion.y = input->travel_duty;
        output->requested_motion.turn = 0.0f;
        output->motion_valid = true;

        follower->lost_elapsed_s = 0.0f;
        follower->status = TAPE_FOLLOWER_TRACKING;
        *active_ever_tracked = true;
    } else {
        /* Search only after this directional sensor has acquired tape. This
         * prevents front-sensor history from affecting backward recovery. */
        reset_controller(follower);
        follower->lost_elapsed_s += dt_s;

        if (!*active_ever_tracked ||
            follower->lost_elapsed_s >= follower->config->lost_timeout_s) {
            follower->status = TAPE_FOLLOWER_LOST;
            output->motion_valid = false;
        } else {
            float search_direction = 0.0f;
            if (active_estimator_state->last_known_error > 0.0f) {
                search_direction = 1.0f;
            } else if (active_estimator_state->last_known_error < 0.0f) {
                search_direction = -1.0f;
            }

            follower->status = TAPE_FOLLOWER_SEARCHING;
            output->requested_motion.x =
                search_direction * follower->config->search_duty;
            output->motion_valid = search_direction != 0.0f;
        }
    }

    output->status = follower->status;
    return ESP_OK;
}
