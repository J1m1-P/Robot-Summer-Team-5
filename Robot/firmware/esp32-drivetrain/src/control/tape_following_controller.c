/* Implements bounded PID correction for tape following. */
#include "control/tape_following_controller.h"

#include <stddef.h>
#include <robot_common/math_utils.h>

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
    if (state == NULL || config == NULL || dt_s <= 0.0f) {
        return 0.0f;
    }

    float proportional_term = config->proportional_gain * error;

    state->integral += error * dt_s;
    state->integral = clamp(state->integral, -config->integral_limit, config->integral_limit);
    float integral_term = config->integral_gain * state->integral;

    float derivative_term = 0.0f;
    if (state->has_previous_error) {
        derivative_term = config->derivative_gain *
                          (error - state->previous_error) / dt_s;
    }
    state->previous_error = error;
    state->has_previous_error = true;

    float correction = proportional_term + integral_term + derivative_term;
    correction = clamp(correction, config->correction_min, config->correction_max);
    return correction;
}
