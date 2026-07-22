/* Coordinates the claw's three VL53L0X sensors on one I2C bus. */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "drivers/time_of_flight/vl53l0x_driver.h"
#include "esp_err.h"
#include <robot_common/i2c_bus.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TOF_SENSOR_COUNT 3U

typedef struct {
    const I2cBusConfig *bus_config;
    const VL53L0XConfig *sensor_configs;
} TofManagerConfig;

typedef struct {
    const TofManagerConfig *config;
    I2cBus bus;
    VL53L0X sensors[TOF_SENSOR_COUNT];
    bool initialized;
    bool ranging;
} TofManager;

esp_err_t tof_manager_init(TofManager *manager,
                           const TofManagerConfig *config);
esp_err_t tof_manager_start(TofManager *manager);
esp_err_t tof_manager_poll(TofManager *manager);
esp_err_t tof_manager_stop(TofManager *manager);
esp_err_t tof_manager_deinit(TofManager *manager);

#ifdef __cplusplus
}
#endif
