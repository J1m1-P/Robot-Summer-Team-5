#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"

#include "communication/i2c/i2c_bus.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L0X_DEFAULT_I2C_ADDRESS  0x29U

typedef enum {
    VL53L0X_PROFILE_DEFAULT = 0,
    VL53L0X_PROFILE_HIGH_SPEED,
    VL53L0X_PROFILE_HIGH_ACCURACY,
    VL53L0X_PROFILE_LONG_RANGE
} VL53L0XProfile;

typedef struct {
    /*
     * Final address assigned to this sensor after initialization.
     */
    uint8_t i2c_address;

    /*
     * Hardware shutdown pin.
     * GPIO_NUM_NC means the pin is not controlled.
     */
    gpio_num_t xshut_pin;

    /*
     * Optional measurement-ready interrupt pin.
     * GPIO_NUM_NC means polling is used.
     */
    gpio_num_t interrupt_pin;

    VL53L0XProfile profile;

    /*
     * 0 means use the profile's default timing budget.
     */
    uint32_t timing_budget_us;

    /*
     * Maximum time to wait for sensor operations.
     */
    uint32_t timeout_ms;
} VL53L0XConfig;

typedef struct {
    const VL53L0XConfig *config;
    I2CBus *bus;

    uint16_t last_distance_mm;
    uint8_t last_range_status;

    int64_t last_update_us;

    bool initialized;
    bool ranging;
    bool measurement_valid;
} VL53L0X;

esp_err_t vl53l0x_init(
    VL53L0X *sensor,
    I2CBus *bus,
    const VL53L0XConfig *config
);

esp_err_t vl53l0x_deinit(VL53L0X *sensor);

esp_err_t vl53l0x_start_continuous(VL53L0X *sensor);

esp_err_t vl53l0x_stop(VL53L0X *sensor);

esp_err_t vl53l0x_read_distance(
    VL53L0X *sensor,
    uint16_t *distance_mm
);

bool vl53l0x_is_measurement_valid(
    const VL53L0X *sensor
);

uint16_t vl53l0x_get_last_distance_mm(
    const VL53L0X *sensor
);

#ifdef __cplusplus
}
#endif