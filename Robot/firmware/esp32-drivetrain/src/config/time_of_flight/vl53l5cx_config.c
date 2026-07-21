#include "config/pin_map.h"
#include "config/time_of_flight/vl53l5cx_config.h"

#define VL53L5CX_DEFAULT_I2C_ADDRESS_7BIT  0x29U
#define VL53L5CX_FRONT_I2C_ADDRESS_7BIT    0x33U

#define VL53L5CX_DEFAULT_RANGING_FREQUENCY_HZ  10U
#define VL53L5CX_DEFAULT_TIMEOUT_MS            100U

const VL53L5CXConfig FRONT_VL53L5CX_CONFIG = {
    .id = VL53L5CX_SENSOR_FRONT,
    .resolution = VL53L5CX_CONFIG_RESOLUTION_4X4,
    .ranging_mode = VL53L5CX_CONFIG_RANGING_MODE_CONTINUOUS,
    .target_order = VL53L5CX_CONFIG_TARGET_CLOSEST,

    // Keep disabled until a dedicated LPn GPIO removes the 0x29 collision.
    .enabled = false,

    .default_i2c_address = VL53L5CX_DEFAULT_I2C_ADDRESS_7BIT,
    .target_i2c_address = VL53L5CX_FRONT_I2C_ADDRESS_7BIT,

    /*
     * No unused ToF control GPIO is currently present in pin_map.h.
     * Assign the real LPn pin and set enabled=true together. GPIO_NUM_NC is not
     * sufficient for resolving the shared 0x29 startup-address collision.
     */
    .lpn_pin = PIN_TOF2_XSHUT,
    .intr_pin = GPIO_NUM_NC,

    .ranging_frequency_hz = VL53L5CX_DEFAULT_RANGING_FREQUENCY_HZ,
    .integration_time_ms = 20U,
    .sharpener_percent = 5U,
    .timeout_ms = VL53L5CX_DEFAULT_TIMEOUT_MS,
    .stale_after_ms = 300U,
};
