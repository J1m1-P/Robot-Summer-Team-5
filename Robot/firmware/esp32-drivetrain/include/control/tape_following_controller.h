/* Declares bounded PID correction for tape following. */
#ifndef TAPE_FOLLOWING_CONTROLLER_H
#define TAPE_FOLLOWING_CONTROLLER_H

#include <stdbool.h>

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
} TapeFollowingControllerConfig;

// Running integral and derivative history for one controller instance.
typedef struct {
    float integral;
    float previous_error;
    bool has_previous_error;
} TapeFollowingControllerState;

// Clears integral and derivative history.
void tape_following_controller_reset(TapeFollowingControllerState *state);

// Computes one bounded PID correction from the current estimated line error.
float tape_following_controller_update(TapeFollowingControllerState *state,
                                       const TapeFollowingControllerConfig *config,
                                       float error,
                                       float dt_s);

#ifdef __cplusplus
}
#endif

#endif /* TAPE_FOLLOWING_CONTROLLER_H */
