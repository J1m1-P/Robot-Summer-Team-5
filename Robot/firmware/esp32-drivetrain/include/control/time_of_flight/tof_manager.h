#pragma once

#include <stdbool.h>

#include "drivers/time_of_flight/vl53l0x_driver.h"
#include "drivers/time_of_flight/vl53l5cx_driver.h"
#include "esp_err.h"
#include <robot_common/i2c_bus.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    I2cBus bus;
    VL53L0X vl53l0x[VL53L0X_SENSOR_COUNT];
    VL53L5CX front_grid;
    bool initialized;
    bool ranging;
} TofManager;

// This object is large because the VL53L5CX ULD owns several kilobytes of work
// buffers. Allocate it statically and zero-initialize it before first use.
// Initialization assigns addresses while other sensors are held in shutdown;
// the always-on right VL53L0X is intentionally initialized first.
esp_err_t tof_manager_init(TofManager *manager);
esp_err_t tof_manager_start(TofManager *manager);
esp_err_t tof_manager_poll(TofManager *manager);
esp_err_t tof_manager_stop(TofManager *manager);
esp_err_t tof_manager_deinit(TofManager *manager);

#ifdef __cplusplus
}
#endif
