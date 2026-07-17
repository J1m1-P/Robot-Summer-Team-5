/* Exposes all drivetrain tape-sensor and tape-behavior configuration. */
#pragma once

#include "control/tape_follower.h"
#include "drivers/tape_sensor_driver.h"
#include "sensing/tape_line_estimator.h"
#include "sensing/tape_task_detection.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Shared multiplexer and individual tape-module hardware assignments. */
extern const TapeSensorMuxConfig TAPE_SENSOR_MUX_CONFIG;
extern const TapeSensorDriverConfig FRONT_TAPE_SENSOR_CONFIG;
extern const TapeSensorDriverConfig BACK_TAPE_SENSOR_CONFIG;
extern const TapeSensorDriverConfig LEFT_TAPE_SENSOR_CONFIG;

/* Channel geometry for the front and back guidance modules. */
extern const TapeLineEstimatorConfig FRONT_TAPE_LINE_ESTIMATOR_CONFIG;
extern const TapeLineEstimatorConfig BACK_TAPE_LINE_ESTIMATOR_CONFIG;

/* Complete behavior configurations for line following and task detection. */
extern const TapeFollowerConfig TAPE_FOLLOWER_CONFIG;
extern const TapeTaskDetectorConfig TAPE_TASK_DETECTOR_CONFIG;

#ifdef __cplusplus
}
#endif

