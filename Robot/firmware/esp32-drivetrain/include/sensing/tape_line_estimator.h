/* Declares weighted tape-line position estimation. */
#ifndef TAPE_LINE_ESTIMATOR_H
#define TAPE_LINE_ESTIMATOR_H

#include <stdbool.h>

#include "drivers/tape_sensor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

// Position weight assigned to each tape sensor channel.
typedef struct {
    float channel_weights[TAPE_SENSOR_CHANNEL_COUNT];
} TapeLineEstimatorConfig;

// Retains the line-presence and fallback-error history used by the estimator.
typedef struct {
    bool line_was_present;
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

#endif /* TAPE_LINE_ESTIMATOR_H */
