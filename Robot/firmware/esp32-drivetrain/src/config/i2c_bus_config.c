#include "config/pin_map.h"
#include "config/i2c_bus_config.h"

const I2cBusConfig SENSOR_I2C_BUS_CONFIG = {
    .port = I2C_NUM_0, 

    .sda_pin = PIN_I2C_SDA, 
    .scl_pin = PIN_I2C_SCL, 

    // Start with 100kHz for testing, could try 400kHz later
    .clock_speed_hz = 100000U, 

    .timeout_ms = 50U, 

    // For testing set to true, set to false for actual operation
    .enable_internal_pullups = false
};