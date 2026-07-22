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

/* Configuration ------------------------------------------------------------ */

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

/* Runtime state ------------------------------------------------------------- */

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

/* Lifecycle ----------------------------------------------------------------- */

/* Initializes, calibrates, and assigns the configured runtime address. */
esp_err_t vl53l0x_driver_init(VL53L0X *sensor, I2cBus *bus,
                              const VL53L0XConfig *config);
/* Stops, restores the boot state where possible, and clears runtime state. */
esp_err_t vl53l0x_driver_deinit(VL53L0X *sensor);
esp_err_t vl53l0x_driver_start(VL53L0X *sensor);
esp_err_t vl53l0x_driver_stop(VL53L0X *sensor);

/* Data ---------------------------------------------------------------------- */

/* Caches one ready sample, or returns ESP_ERR_NOT_FINISHED. */
esp_err_t vl53l0x_driver_read(VL53L0X *sensor);
/* Returns the cached sample while fresh; inspect sample.valid for range status. */
esp_err_t vl53l0x_driver_get_sample(const VL53L0X *sensor,
                                    VL53L0XSample *sample);

#ifdef __cplusplus
}
#endif
