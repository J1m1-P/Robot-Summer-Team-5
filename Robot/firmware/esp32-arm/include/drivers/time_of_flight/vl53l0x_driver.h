/* Single-zone VL53L0X configuration, state, and driver API. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/time_of_flight/tof_device.h"
#include "esp_err.h"
#include <robot_common/i2c_bus.h>
#include <vl53l0x_api.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VL53L0X_PROFILE_DEFAULT = 0,
    VL53L0X_PROFILE_HIGH_SPEED,
    VL53L0X_PROFILE_HIGH_ACCURACY,
    VL53L0X_PROFILE_LONG_RANGE
} VL53L0XProfile;

typedef struct {
    TofDeviceConfig device;
    VL53L0XProfile profile;
    uint32_t timing_budget_us;
    uint32_t stop_timeout_ms;
    uint32_t stale_after_ms;
} VL53L0XConfig;

typedef struct {
    uint16_t distance_mm;
    uint8_t range_status;
    int64_t timestamp_us;
    bool valid;
} VL53L0XSample;

typedef struct {
    const VL53L0XConfig *config;
    I2cDevice i2c_device;
    VL53L0X_Dev_t vendor_device;
    VL53L0XSample sample;
    bool initialized;
    bool ranging;
    bool has_data;
} VL53L0X;

esp_err_t vl53l0x_driver_init(VL53L0X *sensor, I2cBus *bus,
                              const VL53L0XConfig *config);
esp_err_t vl53l0x_driver_deinit(VL53L0X *sensor);
esp_err_t vl53l0x_driver_start(VL53L0X *sensor);
esp_err_t vl53l0x_driver_stop(VL53L0X *sensor);
esp_err_t vl53l0x_driver_read(VL53L0X *sensor);
esp_err_t vl53l0x_driver_get_sample(const VL53L0X *sensor,
                                    VL53L0XSample *sample);

#ifdef __cplusplus
}
#endif
