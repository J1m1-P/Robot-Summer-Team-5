#pragma once

#include <stdint.h>

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// Identifies each VL53L5CX by its physical role on the robot.
typedef enum {
    VL53L5CX_SENSOR_FRONT = 0,
    VL53L5CX_SENSOR_COUNT
} VL53L5CXSensorId;

// Selects the number of independently reported ranging zones.
typedef enum {
    VL53L5CX_RESOLUTION_4X4 = 16,
    VL53L5CX_RESOLUTION_8X8 = 64
} VL53L5CXResolution;

// Selects whether measurements run continuously or at autonomous intervals.
typedef enum {
    VL53L5CX_RANGING_MODE_CONTINUOUS = 0,
    VL53L5CX_RANGING_MODE_AUTONOMOUS
} VL53L5CXRangingMode;

typedef struct {
    VL53L5CXSensorId id;
    VL53L5CXResolution resolution;
    VL53L5CXRangingMode ranging_mode;

    // Project-owned addresses always use the unshifted 7-bit form.
    uint8_t default_i2c_address;
    uint8_t target_i2c_address;

    // GPIO_NUM_NC means the signal is not controlled by this ESP32.
    gpio_num_t lpn_pin;
    gpio_num_t intr_pin;

    uint8_t ranging_frequency_hz;
    uint32_t timeout_ms;
} VL53L5CXConfig;

extern const VL53L5CXConfig FRONT_VL53L5CX_CONFIG;

#ifdef __cplusplus
}
#endif
