#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include <robot_common/i2c_bus.h>
#include <vl53l5cx_api.h>

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L5CX_MAX_ZONES 64U

// Identifies each VL53L5CX by its physical role on the robot.
typedef enum {
    VL53L5CX_SENSOR_FRONT = 0,
    VL53L5CX_SENSOR_COUNT
} VL53L5CXSensorId;

// Selects the number of independently reported ranging zones.
typedef enum {
    VL53L5CX_CONFIG_RESOLUTION_4X4 = 16,
    VL53L5CX_CONFIG_RESOLUTION_8X8 = 64
} VL53L5CXResolution;

// Selects whether measurements run continuously or at autonomous intervals.
typedef enum {
    VL53L5CX_CONFIG_RANGING_MODE_CONTINUOUS = 0,
    VL53L5CX_CONFIG_RANGING_MODE_AUTONOMOUS
} VL53L5CXRangingMode;

typedef enum {
    VL53L5CX_CONFIG_TARGET_CLOSEST = 0,
    VL53L5CX_CONFIG_TARGET_STRONGEST
} VL53L5CXTargetOrder;

typedef struct {
    VL53L5CXSensorId id;
    VL53L5CXResolution resolution;
    VL53L5CXRangingMode ranging_mode;
    VL53L5CXTargetOrder target_order;

    // Disabled entries are intentionally skipped by the ToF manager.
    bool enabled;

    // Project-owned addresses always use the unshifted 7-bit form.
    uint8_t default_i2c_address;
    uint8_t target_i2c_address;

    // GPIO_NUM_NC means the signal is not controlled by this ESP32.
    gpio_num_t lpn_pin;
    gpio_num_t intr_pin;

    uint8_t ranging_frequency_hz;
    uint16_t integration_time_ms;
    uint8_t sharpener_percent;
    uint32_t timeout_ms;
    uint32_t stale_after_ms;
} VL53L5CXConfig;

typedef struct {
    const VL53L5CXConfig *config;
    I2cBus *bus;
    I2cDevice device;
    VL53L5CX_Configuration vendor_device;
    VL53L5CX_ResultsData results;

    uint64_t valid_zone_mask;
    int64_t last_update_us;
    uint32_t frame_count;
    uint8_t zone_count;
    uint8_t last_vendor_status;
    bool initialized;
    bool ranging;
} VL53L5CX;

esp_err_t vl53l5cx_driver_init(VL53L5CX *sensor, I2cBus *bus,
                               const VL53L5CXConfig *config);
esp_err_t vl53l5cx_driver_deinit(VL53L5CX *sensor);
esp_err_t vl53l5cx_driver_start(VL53L5CX *sensor);
esp_err_t vl53l5cx_driver_stop(VL53L5CX *sensor);
esp_err_t vl53l5cx_driver_read(VL53L5CX *sensor);
bool vl53l5cx_driver_zone_is_valid(const VL53L5CX *sensor, uint8_t zone);
bool vl53l5cx_driver_data_is_fresh(const VL53L5CX *sensor);
esp_err_t vl53l5cx_driver_get_distance_mm(const VL53L5CX *sensor,
                                          uint8_t zone,
                                          int16_t *distance_mm);

#ifdef __cplusplus
}
#endif
