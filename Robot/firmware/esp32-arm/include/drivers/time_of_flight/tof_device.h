/* Common I2C address and shutdown settings for time-of-flight sensors. */
#pragma once

#include <stdint.h>

#include "driver/gpio.h"

typedef struct {
    uint8_t default_i2c_address;
    uint8_t target_i2c_address;
    gpio_num_t shutdown_pin;
} TofDeviceConfig;
