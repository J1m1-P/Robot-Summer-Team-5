#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include <robot_common/i2c_bus.h>
#include <vl53l0x_api.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VL53L0X_SENSOR_LEFT = 0,
    VL53L0X_SENSOR_MID,
    VL53L0X_SENSOR_RIGHT,
    VL53L0X_SENSOR_COUNT
} VL53L0XSensorId;

typedef enum {
    VL53L0X_PROFILE_DEFAULT = 0,
    VL53L0X_PROFILE_HIGH_SPEED,
    VL53L0X_PROFILE_HIGH_ACCURACY,
    VL53L0X_PROFILE_LONG_RANGE
} VL53L0XProfile;

typedef struct {
    VL53L0XSensorId id;
    VL53L0XProfile profile;

    uint8_t default_i2c_address;
    uint8_t target_i2c_address;

    gpio_num_t xshut_pin;
    gpio_num_t intr_pin;

    uint32_t timing_budget_us;
    uint32_t timeout_ms;
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
    I2cBus *bus;
    I2cDevice device;
    VL53L0X_Dev_t vendor_device;

    uint16_t last_distance_mm;
    uint8_t last_range_status;
    int64_t last_update_us;

    bool initialized;
    bool ranging;
    bool measurement_valid;
} VL53L0X;

esp_err_t vl53l0x_init(VL53L0X *sensor, I2cBus *bus,
                       const VL53L0XConfig *config);
esp_err_t vl53l0x_deinit(VL53L0X *sensor);
esp_err_t vl53l0x_start_continuous(VL53L0X *sensor);
esp_err_t vl53l0x_stop(VL53L0X *sensor);
esp_err_t vl53l0x_read_distance(VL53L0X *sensor, uint16_t *distance_mm);
esp_err_t vl53l0x_read_sample(VL53L0X *sensor, VL53L0XSample *sample);
bool vl53l0x_is_measurement_valid(const VL53L0X *sensor);
bool vl53l0x_is_measurement_fresh(const VL53L0X *sensor);
uint16_t vl53l0x_get_last_distance_mm(const VL53L0X *sensor);

#ifdef __cplusplus
}
#endif
