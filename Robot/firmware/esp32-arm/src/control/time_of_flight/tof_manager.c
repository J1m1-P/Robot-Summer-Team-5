/* Coordinates shared-bus startup and polling for the claw ToF sensors. */
#include "control/time_of_flight/tof_manager.h"

#include <string.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TOF_SHUTDOWN_DELAY_MS 5U

static bool config_is_valid(const TofManagerConfig *config)
{
    if (config == NULL || config->bus_config == NULL ||
        config->sensor_configs == NULL) {
        return false;
    }

    size_t always_on_count = 0U;
    for (size_t first = 0U; first < TOF_SENSOR_COUNT; ++first) {
        const TofDeviceConfig *a = &config->sensor_configs[first].device;
        if (a->shutdown_pin == GPIO_NUM_NC) ++always_on_count;
        if (a->target_i2c_address == a->default_i2c_address) return false;
        for (size_t second = first + 1U; second < TOF_SENSOR_COUNT; ++second) {
            const TofDeviceConfig *b = &config->sensor_configs[second].device;
            if (a->target_i2c_address == b->target_i2c_address) return false;
        }
    }
    return always_on_count <= 1U;
}

static esp_err_t set_shutdown(gpio_num_t pin, uint32_t level)
{
    if (pin == GPIO_NUM_NC) return ESP_OK;
    const gpio_config_t gpio = {
        .pin_bit_mask = UINT64_C(1) << (uint32_t)pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t error = gpio_config(&gpio);
    return error == ESP_OK ? gpio_set_level(pin, level) : error;
}

static esp_err_t stop_active_sensors(TofManager *manager)
{
    esp_err_t first_error = ESP_OK;
    for (size_t index = 0U; index < TOF_SENSOR_COUNT; ++index) {
        if (!manager->sensors[index].ranging) continue;
        esp_err_t error = vl53l0x_driver_stop(&manager->sensors[index]);
        if (first_error == ESP_OK) first_error = error;
    }
    return first_error;
}

esp_err_t tof_manager_init(TofManager *manager,
                           const TofManagerConfig *config)
{
    if (manager == NULL || !config_is_valid(config)) return ESP_ERR_INVALID_ARG;
    if (manager->initialized) return ESP_ERR_INVALID_STATE;

    memset(manager, 0, sizeof(*manager));
    manager->config = config;

    for (size_t index = 0U; index < TOF_SENSOR_COUNT; ++index) {
        esp_err_t error = set_shutdown(
            config->sensor_configs[index].device.shutdown_pin, 0U);
        if (error != ESP_OK) return error;
    }
    vTaskDelay(pdMS_TO_TICKS(TOF_SHUTDOWN_DELAY_MS));

    esp_err_t error = i2c_bus_init(&manager->bus, config->bus_config);
    if (error != ESP_OK) return error;

    /*
     * TODO: Address assignment is temporarily owned by the ESP. The planned
     * design has the Pi assign addresses and notify the ESP before ranging.
     */
    /* Address the always-on sensor first, then boot each controlled sensor. */
    for (size_t pass = 0U; pass < 2U; ++pass) {
        const bool controlled = pass != 0U;
        for (size_t index = 0U; index < TOF_SENSOR_COUNT; ++index) {
            const VL53L0XConfig *sensor_config = &config->sensor_configs[index];
            if ((sensor_config->device.shutdown_pin != GPIO_NUM_NC) != controlled) {
                continue;
            }
            error = vl53l0x_driver_init(
                &manager->sensors[index], &manager->bus, sensor_config);
            if (error != ESP_OK) goto fail;
        }
    }

    manager->initialized = true;
    return ESP_OK;

fail:
    for (size_t index = TOF_SENSOR_COUNT; index > 0U; --index) {
        VL53L0X *sensor = &manager->sensors[index - 1U];
        if (sensor->initialized) vl53l0x_driver_deinit(sensor);
    }
    for (size_t index = 0U; index < TOF_SENSOR_COUNT; ++index) {
        set_shutdown(config->sensor_configs[index].device.shutdown_pin, 0U);
    }
    i2c_bus_deinit(&manager->bus);
    memset(manager, 0, sizeof(*manager));
    return error;
}

esp_err_t tof_manager_start(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized || manager->ranging) return ESP_ERR_INVALID_STATE;

    for (size_t index = 0U; index < TOF_SENSOR_COUNT; ++index) {
        esp_err_t error = vl53l0x_driver_start(&manager->sensors[index]);
        if (error != ESP_OK) {
            stop_active_sensors(manager);
            return error;
        }
    }
    manager->ranging = true;
    return ESP_OK;
}

esp_err_t tof_manager_poll(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized || !manager->ranging) return ESP_ERR_INVALID_STATE;

    bool updated = false;
    for (size_t index = 0U; index < TOF_SENSOR_COUNT; ++index) {
        esp_err_t error = vl53l0x_driver_read(&manager->sensors[index]);
        if (error == ESP_OK) updated = true;
        else if (error != ESP_ERR_NOT_FINISHED) return error;
    }
    return updated ? ESP_OK : ESP_ERR_NOT_FINISHED;
}

esp_err_t tof_manager_stop(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized || !manager->ranging) return ESP_ERR_INVALID_STATE;

    esp_err_t error = stop_active_sensors(manager);
    manager->ranging = false;
    return error;
}

esp_err_t tof_manager_deinit(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t first_error = manager->ranging ? tof_manager_stop(manager) : ESP_OK;
    for (size_t index = TOF_SENSOR_COUNT; index > 0U; --index) {
        VL53L0X *sensor = &manager->sensors[index - 1U];
        esp_err_t error = vl53l0x_driver_deinit(sensor);
        if (first_error == ESP_OK) first_error = error;
    }
    esp_err_t error = i2c_bus_deinit(&manager->bus);
    if (first_error == ESP_OK) first_error = error;
    memset(manager, 0, sizeof(*manager));
    return first_error;
}
