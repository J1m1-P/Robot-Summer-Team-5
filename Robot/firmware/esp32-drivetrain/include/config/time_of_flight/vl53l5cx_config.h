/* Exposes the drivetrain board's front VL53L5CX configuration. */
#pragma once

#include "drivers/time_of_flight/vl53l5cx_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

// Front-facing grid sensor configuration.
extern const VL53L5CXConfig FRONT_VL53L5CX_CONFIG;

#ifdef __cplusplus
}
#endif
