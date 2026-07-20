/* Declares weighted tape-line position estimation. */
#pragma once

#include <stdbool.h>

#include "drivers/tape_sensor/tape_sensor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

// Position weight assigned to each tape sensor channel.
typedef struct {
    float channel_weights[TAPE_SENSOR_CHANNEL_COUNT];
} TapeLineEstimatorConfig;

// Retains the fallback-error history used by the estimator.
typedef struct {
    float last_known_error;
} TapeLineEstimatorState;

// Clears line-presence and fallback-error history.
void tape_line_estimator_reset(TapeLineEstimatorState *state);

// Computes a weighted channel centroid or directional fallback when tape is lost.
bool tape_line_estimator_compute_error(const TapeSensor *sensor,
                                       const TapeLineEstimatorConfig *config,
                                       TapeLineEstimatorState *state,
                                       float *out_error);

#ifdef __cplusplus
}
#endif
