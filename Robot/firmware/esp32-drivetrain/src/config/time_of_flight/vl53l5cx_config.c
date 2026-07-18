#include "config/time_of_flight/vl53l5cx_config.h"

#define VL53L5CX_DEFAULT_I2C_ADDRESS  0x29U
#define VL53L5CX_FRONT_I2C_ADDRESS    0x32U

#define VL53L5CX_DEFAULT_RANGING_FREQUENCY_HZ  10U
#define VL53L5CX_DEFAULT_TIMEOUT_MS            100U

const VL53L5CXConfig FRONT_VL53L5CX_CONFIG = {
    .id = VL53L5CX_SENSOR_FRONT,
    .resolution = VL53L5CX_RESOLUTION_4X4,
    .ranging_mode = VL53L5CX_RANGING_MODE_CONTINUOUS,

    .default_i2c_address = VL53L5CX_DEFAULT_I2C_ADDRESS,
    .target_i2c_address = VL53L5CX_FRONT_I2C_ADDRESS,

    /*
     * No unused ToF control GPIO is currently present in pin_map.h.
     * Assign the real LPn pin before enabling this sensor on the same bus as
     * another default-address ToF sensor. GPIO_NUM_NC is not sufficient for
     * resolving the shared 0x29 startup-address collision.
     */
    .lpn_pin = GPIO_NUM_NC,
    .intr_pin = GPIO_NUM_NC,

    .ranging_frequency_hz = VL53L5CX_DEFAULT_RANGING_FREQUENCY_HZ,
    .timeout_ms = VL53L5CX_DEFAULT_TIMEOUT_MS,
};
