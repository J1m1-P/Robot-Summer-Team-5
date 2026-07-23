/* Defines the arm board's shared sensor I2C bus. */
#include "config/i2c_bus_config.h"

#include "config/pin_map.h"

const I2cBusConfig SENSOR_I2C_BUS_CONFIG = {
    .port = I2C_NUM_0,
    .sda_pin = PIN_I2C_SDA,
    .scl_pin = PIN_I2C_SCL,
    .clock_speed_hz = 100000U,
    .timeout_ms = 50U,
    .enable_internal_pullups = true,
};
