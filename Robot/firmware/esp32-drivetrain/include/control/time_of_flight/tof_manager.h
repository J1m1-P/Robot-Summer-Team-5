/* Coordinates a configurable set of time-of-flight sensors on one I2C bus. */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "drivers/time_of_flight/vl53l0x_driver.h"
#include "drivers/time_of_flight/vl53l5cx_driver.h"
#include "esp_err.h"
#include <robot_common/i2c_bus.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Describes the sensor topology without assigning robot-specific meanings. */
typedef struct {
    const I2cBusConfig *bus_config;
    const VL53L0XConfig *vl53l0x_configs;
    size_t vl53l0x_count;
    const VL53L5CXConfig *vl53l5cx_configs;
    size_t vl53l5cx_count;
} TofManagerConfig;

/* Supplies exact-size runtime storage owned by the calling application. */
typedef struct {
    VL53L0X *vl53l0x;
    VL53L5CX *vl53l5cx;
} TofManagerStorage;

/*
 * Owns the shared bus and references the caller's sensor storage.
 * Zero-initialize this object before its first initialization.
 */
typedef struct {
    const TofManagerConfig *config;
    TofManagerStorage storage;
    I2cBus bus;
    bool initialized;
    bool ranging;
} TofManager;

/*
 * Holds controllable sensors in shutdown, initializes any always-on sensor
 * first, and then assigns every sensor its unique runtime address.
 */
esp_err_t tof_manager_init(TofManager *manager,
                           const TofManagerConfig *config,
                           const TofManagerStorage *storage);
/* Starts ranging on every initialized sensor, rolling back on failure. */
esp_err_t tof_manager_start(TofManager *manager);
/* Polls all sensors once and reports whether any new data arrived. */
esp_err_t tof_manager_poll(TofManager *manager);
/* Stops every ranging sensor while preserving the first error. */
esp_err_t tof_manager_stop(TofManager *manager);
/* Stops sensors, powers them down where possible, and releases the bus. */
esp_err_t tof_manager_deinit(TofManager *manager);

#ifdef __cplusplus
}
#endif
