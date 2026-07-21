#include "control/time_of_flight/tof_manager.h"

#include <string.h>

#include "config/communication/i2c_bus_config.h"
#include "config/time_of_flight/vl53l0x_config.h"
#include "config/time_of_flight/vl53l5cx_config.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const VL53L0XConfig *const vl53l0x_configs[VL53L0X_SENSOR_COUNT] = {
    &LEFT_VL53L0X_CONFIG,
    &MID_VL53L0X_CONFIG,
    &RIGHT_VL53L0X_CONFIG,
};

static esp_err_t hold_in_shutdown(const VL53L0XConfig *config)
{
    if (config->xshut_pin == GPIO_NUM_NC) return ESP_OK;
    gpio_config_t gpio = {
        .pin_bit_mask = UINT64_C(1) << (uint32_t)config->xshut_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t error = gpio_config(&gpio);
    return error == ESP_OK ? gpio_set_level(config->xshut_pin, 0U) : error;
}

static void cleanup_sensors(TofManager *manager)
{
    if (manager->front_grid.initialized) {
        vl53l5cx_driver_deinit(&manager->front_grid);
    }
    for (int id = VL53L0X_SENSOR_COUNT - 1; id >= 0; --id) {
        if (manager->vl53l0x[id].initialized) {
            vl53l0x_deinit(&manager->vl53l0x[id]);
        }
    }
}

esp_err_t tof_manager_init(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (manager->initialized) return ESP_ERR_INVALID_STATE;
    memset(manager, 0, sizeof(*manager));

    // More than one enabled, always-on device at the shared boot address is
    // electrically ambiguous and cannot be repaired in software.
    if (FRONT_VL53L5CX_CONFIG.enabled &&
        FRONT_VL53L5CX_CONFIG.lpn_pin == GPIO_NUM_NC &&
        RIGHT_VL53L0X_CONFIG.xshut_pin == GPIO_NUM_NC &&
        FRONT_VL53L5CX_CONFIG.default_i2c_address ==
            RIGHT_VL53L0X_CONFIG.default_i2c_address) {
        return ESP_ERR_INVALID_STATE;
    }

    for (size_t id = 0U; id < VL53L0X_SENSOR_COUNT; ++id) {
        esp_err_t error = hold_in_shutdown(vl53l0x_configs[id]);
        if (error != ESP_OK) return error;
    }
    if (FRONT_VL53L5CX_CONFIG.enabled &&
        FRONT_VL53L5CX_CONFIG.lpn_pin != GPIO_NUM_NC) {
        gpio_config_t gpio = {
            .pin_bit_mask = UINT64_C(1) <<
                (uint32_t)FRONT_VL53L5CX_CONFIG.lpn_pin,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        esp_err_t error = gpio_config(&gpio);
        if (error != ESP_OK) return error;
        error = gpio_set_level(FRONT_VL53L5CX_CONFIG.lpn_pin, 0U);
        if (error != ESP_OK) return error;
    }
    vTaskDelay(pdMS_TO_TICKS(5U));

    esp_err_t error = i2c_bus_init(&manager->bus, &SENSOR_I2C_BUS_CONFIG);
    if (error != ESP_OK) return error;

    // The right sensor has no shutdown connection, so move it off 0x29 first.
    const VL53L0XSensorId order[VL53L0X_SENSOR_COUNT] = {
        VL53L0X_SENSOR_RIGHT, VL53L0X_SENSOR_LEFT, VL53L0X_SENSOR_MID
    };
    for (size_t index = 0U; index < VL53L0X_SENSOR_COUNT; ++index) {
        const VL53L0XSensorId id = order[index];
        error = vl53l0x_init(
            &manager->vl53l0x[id], &manager->bus, vl53l0x_configs[id]);
        if (error != ESP_OK) goto fail;
    }
    if (FRONT_VL53L5CX_CONFIG.enabled) {
        error = vl53l5cx_driver_init(
            &manager->front_grid, &manager->bus, &FRONT_VL53L5CX_CONFIG);
        if (error != ESP_OK) goto fail;
    }
    manager->initialized = true;
    return ESP_OK;

fail:
    cleanup_sensors(manager);
    i2c_bus_deinit(&manager->bus);
    memset(manager, 0, sizeof(*manager));
    return error;
}

esp_err_t tof_manager_start(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized || manager->ranging) return ESP_ERR_INVALID_STATE;
    esp_err_t error = ESP_OK;
    for (size_t id = 0U; id < VL53L0X_SENSOR_COUNT; ++id) {
        error = vl53l0x_start_continuous(&manager->vl53l0x[id]);
        if (error != ESP_OK) goto rollback;
    }
    if (manager->front_grid.initialized) {
        error = vl53l5cx_driver_start(&manager->front_grid);
        if (error != ESP_OK) goto rollback;
    }
    manager->ranging = true;
    return ESP_OK;

rollback:
    for (size_t id = 0U; id < VL53L0X_SENSOR_COUNT; ++id) {
        if (manager->vl53l0x[id].ranging) vl53l0x_stop(&manager->vl53l0x[id]);
    }
    return error;
}

esp_err_t tof_manager_poll(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized || !manager->ranging) return ESP_ERR_INVALID_STATE;
    bool updated = false;
    esp_err_t first_error = ESP_OK;
    for (size_t id = 0U; id < VL53L0X_SENSOR_COUNT; ++id) {
        uint16_t distance_mm;
        esp_err_t error = vl53l0x_read_distance(
            &manager->vl53l0x[id], &distance_mm);
        if (error == ESP_OK) updated = true;
        else if (error != ESP_ERR_NOT_FINISHED && first_error == ESP_OK) {
            first_error = error;
        }
    }
    if (manager->front_grid.initialized) {
        esp_err_t error = vl53l5cx_driver_read(&manager->front_grid);
        if (error == ESP_OK) updated = true;
        else if (error != ESP_ERR_NOT_FINISHED && first_error == ESP_OK) {
            first_error = error;
        }
    }
    if (first_error != ESP_OK) return first_error;
    return updated ? ESP_OK : ESP_ERR_NOT_FINISHED;
}

esp_err_t tof_manager_stop(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized || !manager->ranging) return ESP_ERR_INVALID_STATE;
    esp_err_t first_error = ESP_OK;
    if (manager->front_grid.ranging) {
        first_error = vl53l5cx_driver_stop(&manager->front_grid);
    }
    for (size_t id = 0U; id < VL53L0X_SENSOR_COUNT; ++id) {
        esp_err_t error = vl53l0x_stop(&manager->vl53l0x[id]);
        if (first_error == ESP_OK && error != ESP_OK) first_error = error;
    }
    manager->ranging = false;
    return first_error;
}

esp_err_t tof_manager_deinit(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized) return ESP_ERR_INVALID_STATE;
    esp_err_t first_error = manager->ranging ? tof_manager_stop(manager) : ESP_OK;
    cleanup_sensors(manager);
    esp_err_t bus_error = i2c_bus_deinit(&manager->bus);
    if (first_error == ESP_OK) first_error = bus_error;
    memset(manager, 0, sizeof(*manager));
    return first_error;
}
