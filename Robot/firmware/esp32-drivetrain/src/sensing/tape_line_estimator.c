/* Implements weighted tape-line position estimation. */
#include "sensing/tape_line_estimator.h"

#include <stddef.h>

void tape_line_estimator_reset(TapeLineEstimatorState *state)
{
    if (state == NULL) {
        return;
    }
    state->line_was_present = false;
    state->last_known_error = 0.0f;
}

bool tape_line_estimator_compute_error(const TapeSensor *sensor,
                                       const TapeLineEstimatorConfig *config,
                                       TapeLineEstimatorState *state,
                                       float *out_error)
{
    if (sensor == NULL || config == NULL || state == NULL || out_error == NULL) {
        return false;
    }

    float weighted_sum = 0.0f;
    int active_count = 0;
    if (sensor->channel_0) {
        weighted_sum += config->channel_weights[0];
        active_count++;
    }
    if (sensor->channel_1) {
        weighted_sum += config->channel_weights[1];
        active_count++;
    }
    if (sensor->channel_2) {
        weighted_sum += config->channel_weights[2];
        active_count++;
    }
    if (sensor->channel_3) {
        weighted_sum += config->channel_weights[3];
        active_count++;
    }

    if (active_count > 0) {
        *out_error = weighted_sum / (float)active_count;
        state->last_known_error = *out_error;
        state->line_was_present = true;
        return true;
    }

    *out_error = (state->last_known_error >= 0.0f)
                     ? config->channel_weights[TAPE_SENSOR_CHANNEL_COUNT - 1]
                     : config->channel_weights[0];
    state->line_was_present = false;
    return false;
}
