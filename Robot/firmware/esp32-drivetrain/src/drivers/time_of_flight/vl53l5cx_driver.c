#include "drivers/time_of_flight/vl53l5cx_driver.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define VL53L5CX_BOOT_DELAY_MS 10U

static esp_err_t convert_status(uint8_t status)
{
    switch (status) {
        case VL53L5CX_STATUS_OK: return ESP_OK;
        case VL53L5CX_STATUS_TIMEOUT_ERROR: return ESP_ERR_TIMEOUT;
        case VL53L5CX_STATUS_INVALID_PARAM: return ESP_ERR_INVALID_ARG;
        case VL53L5CX_STATUS_CORRUPTED_FRAME:
        case VL53L5CX_STATUS_CRC_CSUM_FAILED: return ESP_ERR_INVALID_RESPONSE;
        default: return ESP_FAIL;
    }
}

static bool config_is_valid(const VL53L5CXConfig *config)
{
    const bool resolution_ok =
        config != NULL &&
        (config->resolution == VL53L5CX_CONFIG_RESOLUTION_4X4 ||
         config->resolution == VL53L5CX_CONFIG_RESOLUTION_8X8);
    const uint8_t max_frequency = config != NULL &&
        config->resolution == VL53L5CX_CONFIG_RESOLUTION_8X8 ? 15U : 60U;
    return resolution_ok && config->id < VL53L5CX_SENSOR_COUNT &&
           config->ranging_mode <= VL53L5CX_CONFIG_RANGING_MODE_AUTONOMOUS &&
           config->target_order <= VL53L5CX_CONFIG_TARGET_STRONGEST &&
           config->default_i2c_address > 0U &&
           config->default_i2c_address <= 0x7FU &&
           config->target_i2c_address > 0U &&
           config->target_i2c_address <= 0x7FU &&
           config->ranging_frequency_hz > 0U &&
           config->ranging_frequency_hz <= max_frequency &&
           config->integration_time_ms >= 2U &&
           config->integration_time_ms <= 1000U &&
           config->sharpener_percent <= 99U && config->timeout_ms > 0U &&
           config->stale_after_ms > 0U;
}

static esp_err_t set_lpn(const VL53L5CXConfig *config, uint32_t level)
{
    if (config->lpn_pin == GPIO_NUM_NC) return ESP_OK;
    gpio_config_t gpio = {
        .pin_bit_mask = 1ULL << (uint32_t)config->lpn_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t error = gpio_config(&gpio);
    if (error == ESP_OK) error = gpio_set_level(config->lpn_pin, level);
    if (error == ESP_OK) vTaskDelay(pdMS_TO_TICKS(VL53L5CX_BOOT_DELAY_MS));
    return error;
}

static uint8_t apply_config(VL53L5CX *sensor)
{
    const VL53L5CXConfig *config = sensor->config;
    uint8_t status = vl53l5cx_set_resolution(
        &sensor->vendor_device, (uint8_t)config->resolution);
    if (status == 0U) status = vl53l5cx_set_ranging_mode(
        &sensor->vendor_device,
        config->ranging_mode == VL53L5CX_CONFIG_RANGING_MODE_CONTINUOUS
            ? VL53L5CX_RANGING_MODE_CONTINUOUS
            : VL53L5CX_RANGING_MODE_AUTONOMOUS);
    if (status == 0U) status = vl53l5cx_set_ranging_frequency_hz(
        &sensor->vendor_device, config->ranging_frequency_hz);
    if (status == 0U &&
        config->ranging_mode == VL53L5CX_CONFIG_RANGING_MODE_AUTONOMOUS) {
        status = vl53l5cx_set_integration_time_ms(
            &sensor->vendor_device, config->integration_time_ms);
    }
    if (status == 0U) status = vl53l5cx_set_sharpener_percent(
        &sensor->vendor_device, config->sharpener_percent);
    if (status == 0U) status = vl53l5cx_set_target_order(
        &sensor->vendor_device,
        config->target_order == VL53L5CX_CONFIG_TARGET_CLOSEST
            ? VL53L5CX_TARGET_ORDER_CLOSEST
            : VL53L5CX_TARGET_ORDER_STRONGEST);
    return status;
}

esp_err_t vl53l5cx_driver_init(VL53L5CX *sensor, I2cBus *bus,
                               const VL53L5CXConfig *config)
{
    if (sensor == NULL || bus == NULL || !config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sensor->initialized || !i2c_bus_is_initialized(bus)) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(sensor, 0, sizeof(*sensor));
    sensor->config = config;
    sensor->bus = bus;
    esp_err_t error = set_lpn(config, 1U);
    if (error != ESP_OK) goto fail;
    error = i2c_device_init(&sensor->device, bus, config->default_i2c_address);
    if (error != ESP_OK) goto fail;

    sensor->vendor_device.platform.address =
        (uint16_t)(config->default_i2c_address << 1U);
    sensor->vendor_device.platform.i2c_device = &sensor->device;
    uint8_t alive = 0U;
    uint8_t status = vl53l5cx_is_alive(&sensor->vendor_device, &alive);
    if (status != 0U || alive == 0U) {
        error = status != 0U ? convert_status(status) : ESP_ERR_NOT_FOUND;
        goto fail;
    }
    status = vl53l5cx_init(&sensor->vendor_device);
    if (status != 0U) goto vendor_fail;
    if (config->target_i2c_address != config->default_i2c_address) {
        status = vl53l5cx_set_i2c_address(
            &sensor->vendor_device,
            (uint16_t)(config->target_i2c_address << 1U));
        if (status != 0U) goto vendor_fail;
        sensor->device.address = config->target_i2c_address;
    }
    status = apply_config(sensor);
    if (status != 0U) goto vendor_fail;

    sensor->zone_count = (uint8_t)config->resolution;
    sensor->initialized = true;
    return ESP_OK;

vendor_fail:
    sensor->last_vendor_status = status;
    error = convert_status(status);
fail:
    if (config->lpn_pin != GPIO_NUM_NC) gpio_set_level(config->lpn_pin, 0U);
    memset(sensor, 0, sizeof(*sensor));
    return error;
}

esp_err_t vl53l5cx_driver_deinit(VL53L5CX *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized) return ESP_ERR_INVALID_STATE;
    esp_err_t error = sensor->ranging ? vl53l5cx_driver_stop(sensor) : ESP_OK;
    if (sensor->config->lpn_pin != GPIO_NUM_NC) {
        esp_err_t pin_error = gpio_set_level(sensor->config->lpn_pin, 0U);
        if (error == ESP_OK) error = pin_error;
    }
    memset(sensor, 0, sizeof(*sensor));
    return error;
}

esp_err_t vl53l5cx_driver_start(VL53L5CX *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized || sensor->ranging) return ESP_ERR_INVALID_STATE;
    uint8_t status = vl53l5cx_start_ranging(&sensor->vendor_device);
    sensor->last_vendor_status = status;
    if (status != 0U) return convert_status(status);
    sensor->ranging = true;
    sensor->valid_zone_mask = 0U;
    return ESP_OK;
}

esp_err_t vl53l5cx_driver_stop(VL53L5CX *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized || !sensor->ranging) return ESP_ERR_INVALID_STATE;
    uint8_t status = vl53l5cx_stop_ranging(&sensor->vendor_device);
    sensor->last_vendor_status = status;
    if (status != 0U) return convert_status(status);
    sensor->ranging = false;
    sensor->valid_zone_mask = 0U;
    return ESP_OK;
}

esp_err_t vl53l5cx_driver_read(VL53L5CX *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized || !sensor->ranging) return ESP_ERR_INVALID_STATE;
    uint8_t ready = 0U;
    uint8_t status = vl53l5cx_check_data_ready(&sensor->vendor_device, &ready);
    if (status != 0U) return convert_status(status);
    if (ready == 0U) return ESP_ERR_NOT_FINISHED;
    status = vl53l5cx_get_ranging_data(
        &sensor->vendor_device, &sensor->results);
    sensor->last_vendor_status = status;
    if (status != 0U) return convert_status(status);

    uint64_t valid_mask = 0U;
    for (uint8_t zone = 0U; zone < sensor->zone_count; ++zone) {
        const uint8_t target_status = sensor->results.target_status[zone];
        if (sensor->results.nb_target_detected[zone] > 0U &&
            (target_status == 5U || target_status == 9U)) {
            valid_mask |= UINT64_C(1) << zone;
        }
    }
    sensor->valid_zone_mask = valid_mask;
    sensor->last_update_us = esp_timer_get_time();
    ++sensor->frame_count;
    return ESP_OK;
}

bool vl53l5cx_driver_zone_is_valid(const VL53L5CX *sensor, uint8_t zone)
{
    return sensor != NULL && sensor->initialized && zone < sensor->zone_count &&
           (sensor->valid_zone_mask & (UINT64_C(1) << zone)) != 0U;
}

bool vl53l5cx_driver_data_is_fresh(const VL53L5CX *sensor)
{
    return sensor != NULL && sensor->initialized && sensor->frame_count > 0U &&
           esp_timer_get_time() - sensor->last_update_us <=
               (int64_t)sensor->config->stale_after_ms * 1000LL;
}

esp_err_t vl53l5cx_driver_get_distance_mm(const VL53L5CX *sensor,
                                          uint8_t zone,
                                          int16_t *distance_mm)
{
    if (sensor == NULL || distance_mm == NULL || zone >= VL53L5CX_MAX_ZONES) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!vl53l5cx_driver_zone_is_valid(sensor, zone) ||
        !vl53l5cx_driver_data_is_fresh(sensor)) {
        return ESP_ERR_INVALID_STATE;
    }
    *distance_mm = sensor->results.distance_mm[zone];
    return ESP_OK;
}
