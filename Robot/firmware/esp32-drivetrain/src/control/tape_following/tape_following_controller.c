/* Implements bounded PID correction for tape following. */
#include "control/tape_following/tape_following_controller.h"

#include <math.h>
#include <stddef.h>

static float clamp_value(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

void tape_following_controller_reset(TapeFollowingControllerState *state)
{
    if (state == NULL) {
        return;
    }
    state->integral = 0.0f;
    state->previous_error = 0.0f;
    state->has_previous_error = false;
}

float tape_following_controller_update(TapeFollowingControllerState *state,
                                       const TapeFollowingControllerConfig *config,
                                       float error,
                                       float dt_s)
{
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

    float proportional_term = config->proportional_gain * error;

    state->integral += error * dt_s;
    state->integral = clamp_value(
        state->integral, -config->integral_limit, config->integral_limit);
    float integral_term = config->integral_gain * state->integral;

    float derivative_term = 0.0f;
    if (state->has_previous_error) {
        derivative_term = config->derivative_gain *
                          (error - state->previous_error) / dt_s;
    }
    state->previous_error = error;
    state->has_previous_error = true;

    float correction = proportional_term + integral_term + derivative_term;
    correction = clamp_value(
        correction, config->correction_min, config->correction_max);
    return correction;
}
