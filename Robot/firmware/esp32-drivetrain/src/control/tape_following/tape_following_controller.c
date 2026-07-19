/* Implements bounded PID correction for tape following. */
#include "control/tape_following/tape_following_controller.h"

#include <math.h>
#include <stddef.h>
#include <robot_common/math_utils.h>

esp_err_t tape_following_controller_reset(TapeFollowingControllerState *state) {
    if (state == NULL) return ESP_ERR_INVALID_ARG;
    state->integral = 0.0f;
    state->previous_error = 0.0f;
    state->has_previous_error = false;
}

float tape_following_controller_update(TapeFollowingControllerState *state,
                                       const TapeFollowingControllerConfig *config,
                                       float error,
                                       float dt_s)
{
    // Failure check
    if (state == NULL || config == NULL || !isfinite(error) ||
        !isfinite(dt_s) || dt_s <= 0.0f ||
        !isfinite(config->proportional_gain) ||
        !isfinite(config->integral_gain) ||
        !isfinite(config->derivative_gain) ||
        !isfinite(config->integral_limit) || config->integral_limit < 0.0f ||
        !isfinite(config->correction_min) ||
        !isfinite(config->correction_max) ||
        config->correction_min > config->correction_max) {
        return 0.0f;
    }

    // P
    float proportional_term = config->proportional_gain * error;

    // I
    state->integral += error * dt_s;
    state->integral = clamp(state->integral, -config->integral_limit, config->integral_limit);
    float integral_term = config->integral_gain * state->integral;

    // D
    float derivative_term = 0.0f;
    if (state->has_previous_error) {
        derivative_term = config->derivative_gain *
                          (error - state->previous_error) / dt_s;
    }

    // Update the state
    state->previous_error = error;
    state->has_previous_error = true;

    // Output the final correction
    float correction = proportional_term + integral_term + derivative_term;
    correction = clamp(correction, config->correction_min, config->correction_max);

    return correction;
}
