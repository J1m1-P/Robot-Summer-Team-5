/* Implements bounded PID correction for tape following. */
#include "control/tape_following/tape_following_controller.h"

#include <math.h>
#include <stddef.h>
#include <robot_common/math_utils.h>

bool tape_following_controller_config_is_valid(
    const TapeFollowingControllerConfig *config)
{
    return config != NULL &&
           isfinite(config->proportional_gain) &&
           isfinite(config->integral_gain) &&
           isfinite(config->derivative_gain) &&
           isfinite(config->integral_limit) && config->integral_limit >= 0.0f &&
           isfinite(config->correction_min) &&
           isfinite(config->correction_max) &&
           config->correction_min <= config->correction_max;
}

esp_err_t tape_following_controller_reset(TapeFollowingControllerState *state) {
    if (state == NULL) return ESP_ERR_INVALID_ARG;
    state->integral = 0.0f;
    state->previous_error = 0.0f;
    state->has_previous_error = false;
    return ESP_OK;
}

float tape_following_controller_update(TapeFollowingControllerState *state,
                                       const TapeFollowingControllerConfig *config,
                                       float error,
                                       float dt_s)
{
    if (state == NULL || !tape_following_controller_config_is_valid(config) ||
        !isfinite(error) || !isfinite(dt_s) || dt_s <= 0.0f) {
        return 0.0f;
    }

    const float proportional_term = config->proportional_gain * error;

    // I
    state->integral += error * dt_s;
    state->integral = clamp(state->integral, -config->integral_limit, config->integral_limit);
    const float integral_term = config->integral_gain * state->integral;

    // D
    float derivative_term = 0.0f;
    if (state->has_previous_error) {
        derivative_term = config->derivative_gain *
                          (error - state->previous_error) / dt_s;
    }

    state->previous_error = error;
    state->has_previous_error = true;

    return clamp(proportional_term + integral_term + derivative_term,
                 config->correction_min, config->correction_max);
}
