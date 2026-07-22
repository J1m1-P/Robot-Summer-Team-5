/* Multi-zone VL53L5CX configuration, state, and driver API. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "drivers/time_of_flight/tof_device.h"
#include "esp_err.h"
#include <robot_common/i2c_bus.h>
#include <vl53l5cx_api.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration ------------------------------------------------------------ */

typedef enum {
    VL53L5CX_CONFIG_RESOLUTION_4X4 = 16,
    VL53L5CX_CONFIG_RESOLUTION_8X8 = 64
} VL53L5CXResolution;

typedef enum {
    VL53L5CX_CONFIG_RANGING_MODE_CONTINUOUS = 0,
    VL53L5CX_CONFIG_RANGING_MODE_AUTONOMOUS
} VL53L5CXRangingMode;

typedef enum {
    VL53L5CX_CONFIG_TARGET_CLOSEST = 0,
    VL53L5CX_CONFIG_TARGET_STRONGEST
} VL53L5CXTargetOrder;

typedef struct {
    TofDeviceConfig device;
    VL53L5CXResolution resolution;
    VL53L5CXRangingMode ranging_mode;
    VL53L5CXTargetOrder target_order;
    uint8_t ranging_frequency_hz;
    uint16_t integration_time_ms;  // Used only in autonomous mode.
    uint8_t sharpener_percent;
    uint32_t stale_after_ms;
} VL53L5CXConfig;

/* Runtime state ------------------------------------------------------------- */

typedef struct {
    const VL53L5CXConfig *config;
    I2cDevice i2c_device;
    VL53L5CX_Configuration vendor_device;
    VL53L5CX_ResultsData results;
    uint64_t valid_zone_mask;  // Bit N marks zone N as valid.
    int64_t last_update_us;
    bool initialized;
    bool ranging;
    bool has_data;
} VL53L5CX;

/* Lifecycle ----------------------------------------------------------------- */

/* Initializes the ULD and assigns the configured runtime address. */
esp_err_t vl53l5cx_driver_init(VL53L5CX *sensor, I2cBus *bus,
                               const VL53L5CXConfig *config);
/* Stops, restores the boot state where possible, and clears runtime state. */
esp_err_t vl53l5cx_driver_deinit(VL53L5CX *sensor);
esp_err_t vl53l5cx_driver_start(VL53L5CX *sensor);
esp_err_t vl53l5cx_driver_stop(VL53L5CX *sensor);

/* Data ---------------------------------------------------------------------- */

/* Caches one ready frame, or returns ESP_ERR_NOT_FINISHED. */
esp_err_t vl53l5cx_driver_read(VL53L5CX *sensor);
/* Returns one cached distance only while its zone is valid and fresh. */
esp_err_t vl53l5cx_driver_get_distance_mm(const VL53L5CX *sensor,
                                          uint8_t zone,
                                          int16_t *distance_mm);

#ifdef __cplusplus
}
#endif
