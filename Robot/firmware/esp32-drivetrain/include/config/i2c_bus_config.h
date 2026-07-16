/* Exposes the drivetrain board's sensor I2C bus configuration. */
#pragma once

#include <robot_common/i2c_bus.h>

#ifdef __cplusplus
extern "C" {
#endif

// Sensor bus configuration using the drivetrain board's I2C pins.
extern const I2cBusConfig SENSOR_I2C_BUS_CONFIG;

#ifdef __cplusplus
}
#endif
