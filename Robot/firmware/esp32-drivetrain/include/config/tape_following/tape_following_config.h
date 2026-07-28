/* Exposes tape-sensor hardware configuration. */
#pragma once

#include "drivers/tape_sensor/tape_sensor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Shared multiplexer and individual tape-module hardware assignments. */
extern const TapeSensorMuxConfig TAPE_SENSOR_MUX_CONFIG;
extern const TapeSensorDriverConfig FRONT_TAPE_SENSOR_CONFIG;
extern const TapeSensorDriverConfig BACK_TAPE_SENSOR_CONFIG;
extern const TapeSensorDriverConfig LEFT_TAPE_SENSOR_CONFIG;

#ifdef __cplusplus
}
#endif
