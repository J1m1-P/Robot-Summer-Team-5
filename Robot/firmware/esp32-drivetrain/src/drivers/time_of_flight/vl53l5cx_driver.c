/* Implements lifecycle and cached multi-zone ranging for VL53L5CX sensors. */
#include "drivers/time_of_flight/vl53l5cx_driver.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define VL53L5CX_BOOT_DELAY_MS 10U
#define VL53L5CX_VALID_TARGET_STATUS            5U
#define VL53L5CX_VALID_TARGET_STATUS_LARGE_PULSE 9U

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
    if (config == NULL) return false;
    const bool resolution_is_8x8 =
        config->resolution == VL53L5CX_CONFIG_RESOLUTION_8X8;
    const bool resolution_is_valid = resolution_is_8x8 ||
        config->resolution == VL53L5CX_CONFIG_RESOLUTION_4X4;
    const uint8_t max_frequency = resolution_is_8x8 ? 15U : 60U;
    const bool integration_is_valid =
        config->ranging_mode == VL53L5CX_CONFIG_RANGING_MODE_CONTINUOUS ||
        (config->integration_time_ms >= 2U &&
         config->integration_time_ms <= 1000U);

    return resolution_is_valid &&
           config->ranging_mode <= VL53L5CX_CONFIG_RANGING_MODE_AUTONOMOUS &&
           config->target_order <= VL53L5CX_CONFIG_TARGET_STRONGEST &&
           config->device.default_i2c_address > 0U &&
           config->device.default_i2c_address <= 0x7FU &&
           config->device.target_i2c_address > 0U &&
           config->device.target_i2c_address <= 0x7FU &&
           config->ranging_frequency_hz > 0U &&
           config->ranging_frequency_hz <= max_frequency &&
           integration_is_valid && config->sharpener_percent <= 99U &&
           config->stale_after_ms > 0U;
}

/* Drives LPn when controlled and waits for the power-state transition. */
static esp_err_t power_on(const TofDeviceConfig *config)
{
    if (config->shutdown_pin == GPIO_NUM_NC) return ESP_OK;
    const gpio_config_t gpio = {
        .pin_bit_mask = UINT64_C(1) << (uint32_t)config->shutdown_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t error = gpio_config(&gpio);
    if (error == ESP_OK) error = gpio_set_level(config->shutdown_pin, 1U);
    if (error == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(VL53L5CX_BOOT_DELAY_MS));
    }
    return error;
}

/* Updates the hardware and project transport address together. */
static uint8_t set_address(VL53L5CX *sensor, uint8_t address)
{
    uint8_t status = vl53l5cx_set_i2c_address(
        &sensor->vendor_device, (uint16_t)(address << 1U));
    if (status == VL53L5CX_STATUS_OK) sensor->i2c_device.address = address;
    return status;
}

static uint8_t apply_config(VL53L5CX *sensor)
{
    const VL53L5CXConfig *config = sensor->config;
    uint8_t status = vl53l5cx_set_resolution(
        &sensor->vendor_device, (uint8_t)config->resolution);
    if (status == VL53L5CX_STATUS_OK) {
        status = vl53l5cx_set_ranging_mode(
            &sensor->vendor_device,
            config->ranging_mode == VL53L5CX_CONFIG_RANGING_MODE_CONTINUOUS
                ? VL53L5CX_RANGING_MODE_CONTINUOUS
                : VL53L5CX_RANGING_MODE_AUTONOMOUS);
    }
    if (status == VL53L5CX_STATUS_OK) {
        status = vl53l5cx_set_ranging_frequency_hz(
            &sensor->vendor_device, config->ranging_frequency_hz);
    }
    if (status == VL53L5CX_STATUS_OK &&
        config->ranging_mode == VL53L5CX_CONFIG_RANGING_MODE_AUTONOMOUS) {
        status = vl53l5cx_set_integration_time_ms(
            &sensor->vendor_device, config->integration_time_ms);
    }
    if (status == VL53L5CX_STATUS_OK) {
        status = vl53l5cx_set_sharpener_percent(
            &sensor->vendor_device, config->sharpener_percent);
    }
    if (status == VL53L5CX_STATUS_OK) {
        status = vl53l5cx_set_target_order(
            &sensor->vendor_device,
            config->target_order == VL53L5CX_CONFIG_TARGET_CLOSEST
                ? VL53L5CX_TARGET_ORDER_CLOSEST
                : VL53L5CX_TARGET_ORDER_STRONGEST);
    }
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
    bool address_changed = false;

    esp_err_t error = power_on(&config->device);
    if (error != ESP_OK) goto fail;
    error = i2c_device_init(&sensor->i2c_device, bus,
                            config->device.default_i2c_address);
    if (error != ESP_OK) goto fail;

    sensor->vendor_device.platform.address =
        (uint16_t)(config->device.default_i2c_address << 1U);
    sensor->vendor_device.platform.i2c_device = &sensor->i2c_device;

    uint8_t alive = 0U;
    uint8_t status = vl53l5cx_is_alive(&sensor->vendor_device, &alive);
    if (status != VL53L5CX_STATUS_OK || alive == 0U) {
        error = status == VL53L5CX_STATUS_OK
            ? ESP_ERR_NOT_FOUND : convert_status(status);
        goto fail;
    }

    status = vl53l5cx_init(&sensor->vendor_device);
    if (status != VL53L5CX_STATUS_OK) goto vendor_fail;
    if (config->device.target_i2c_address !=
        config->device.default_i2c_address) {
        status = set_address(sensor, config->device.target_i2c_address);
        if (status != VL53L5CX_STATUS_OK) goto vendor_fail;
        address_changed = true;
    }
    status = apply_config(sensor);
    if (status != VL53L5CX_STATUS_OK) goto vendor_fail;

    sensor->initialized = true;
    return ESP_OK;

vendor_fail:
    error = convert_status(status);
fail:
    if (config->device.shutdown_pin != GPIO_NUM_NC) {
        gpio_set_level(config->device.shutdown_pin, 0U);
    } else if (address_changed) {
        set_address(sensor, config->device.default_i2c_address);
    }
    memset(sensor, 0, sizeof(*sensor));
    return error;
}

esp_err_t vl53l5cx_driver_deinit(VL53L5CX *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t first_error = sensor->ranging
        ? vl53l5cx_driver_stop(sensor) : ESP_OK;
    if (sensor->config->device.shutdown_pin != GPIO_NUM_NC) {
        esp_err_t pin_error = gpio_set_level(
            sensor->config->device.shutdown_pin, 0U);
        if (first_error == ESP_OK) first_error = pin_error;
    } else if (sensor->config->device.target_i2c_address !=
               sensor->config->device.default_i2c_address) {
        uint8_t status = set_address(
            sensor, sensor->config->device.default_i2c_address);
        if (first_error == ESP_OK) first_error = convert_status(status);
    }
    memset(sensor, 0, sizeof(*sensor));
    return first_error;
}

esp_err_t vl53l5cx_driver_start(VL53L5CX *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized || sensor->ranging) return ESP_ERR_INVALID_STATE;

    uint8_t status = vl53l5cx_start_ranging(&sensor->vendor_device);
    if (status != VL53L5CX_STATUS_OK) return convert_status(status);
    sensor->valid_zone_mask = 0U;
    sensor->has_data = false;
    sensor->ranging = true;
    return ESP_OK;
}

esp_err_t vl53l5cx_driver_stop(VL53L5CX *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized || !sensor->ranging) return ESP_ERR_INVALID_STATE;

    uint8_t status = vl53l5cx_stop_ranging(&sensor->vendor_device);
    if (status != VL53L5CX_STATUS_OK) return convert_status(status);
    sensor->valid_zone_mask = 0U;
    sensor->has_data = false;
    sensor->ranging = false;
    return ESP_OK;
}

esp_err_t vl53l5cx_driver_read(VL53L5CX *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized || !sensor->ranging) return ESP_ERR_INVALID_STATE;

    uint8_t ready = 0U;
    uint8_t status = vl53l5cx_check_data_ready(&sensor->vendor_device, &ready);
    if (status != VL53L5CX_STATUS_OK) return convert_status(status);
    if (ready == 0U) return ESP_ERR_NOT_FINISHED;

    status = vl53l5cx_get_ranging_data(
        &sensor->vendor_device, &sensor->results);
    if (status != VL53L5CX_STATUS_OK) return convert_status(status);

    uint64_t valid_mask = 0U;
    for (uint8_t zone = 0U; zone < (uint8_t)sensor->config->resolution; ++zone) {
        const uint8_t target_status = sensor->results.target_status[zone];
        if (sensor->results.nb_target_detected[zone] > 0U &&
            (target_status == VL53L5CX_VALID_TARGET_STATUS ||
             target_status == VL53L5CX_VALID_TARGET_STATUS_LARGE_PULSE)) {
            valid_mask |= UINT64_C(1) << zone;
        }
    }
    sensor->valid_zone_mask = valid_mask;
    sensor->last_update_us = esp_timer_get_time();
    sensor->has_data = true;
    return ESP_OK;
}

esp_err_t vl53l5cx_driver_get_distance_mm(const VL53L5CX *sensor,
                                          uint8_t zone,
                                          int16_t *distance_mm)
{
    if (sensor == NULL || distance_mm == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized || zone >= (uint8_t)sensor->config->resolution) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!sensor->has_data ||
        (sensor->valid_zone_mask & (UINT64_C(1) << zone)) == 0U ||
        esp_timer_get_time() - sensor->last_update_us >
            (int64_t)sensor->config->stale_after_ms * 1000LL) {
        return ESP_ERR_INVALID_STATE;
    }
    *distance_mm = sensor->results.distance_mm[zone];
    return ESP_OK;
}
