/* Exposes tape sensor pin mappings and PID position weights. */
#pragma once


#include "sensors/tape_following.h"

#ifdef __cplusplus
extern "C" {
#endif

// Pin configurations for the front, back, and left tape sensor modules.
extern const TapeSensorConfig TAPE_SENSOR_FRONT;
extern const TapeSensorConfig TAPE_SENSOR_BACK;
extern const TapeSensorConfig TAPE_SENSOR_LEFT;

#ifdef __cplusplus
}
#endif
