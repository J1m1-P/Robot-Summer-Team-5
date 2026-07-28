/* Declares a generic bounded PID correction for velocity/heading control. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Tunable PID gains and correction limits.
typedef struct {
    float proportional_gain;
    float integral_gain;
    float derivative_gain;
    float integral_limit;
    float correction_min;
    float correction_max;
} BoundedPidConfig;

// Running integral and derivative history for one controller instance.
typedef struct {
    float integral;
    float previous_error;
    bool has_previous_error;
} BoundedPidState;

// Checks gains and limits before they enter a running controller.
bool bounded_pid_config_is_valid(const BoundedPidConfig *config);

// Clears integral and derivative history.
esp_err_t bounded_pid_reset(BoundedPidState *state);

// Computes one bounded PID correction from the current error.
float bounded_pid_update(BoundedPidState *state,
                         const BoundedPidConfig *config,
                         float error,
                         float dt_s);

#ifdef __cplusplus
}
#endif
