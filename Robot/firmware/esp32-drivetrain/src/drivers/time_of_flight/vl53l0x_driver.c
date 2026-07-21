#include "drivers/time_of_flight/vl53l0x_driver.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define VL53L0X_BOOT_DELAY_MS  5U
#define VL53L0X_ADDRESS_DELAY_MS  10U

static esp_err_t vl53l0x_convert_error(VL53L0X_Error error)
{
    switch (error) {
        case VL53L0X_ERROR_NONE:
            return ESP_OK;
        case VL53L0X_ERROR_INVALID_PARAMS:
            return ESP_ERR_INVALID_ARG;
        case VL53L0X_ERROR_TIME_OUT:
            return ESP_ERR_TIMEOUT;
        case VL53L0X_ERROR_NOT_SUPPORTED:
        case VL53L0X_ERROR_MODE_NOT_SUPPORTED:
            return ESP_ERR_NOT_SUPPORTED;
        default:
            return ESP_FAIL;
    }
}

static bool vl53l0x_config_is_valid(const VL53L0XConfig *config)
{
    return config != NULL &&
           config->id >= VL53L0X_SENSOR_LEFT &&
           config->id < VL53L0X_SENSOR_COUNT &&
           config->profile >= VL53L0X_PROFILE_DEFAULT &&
           config->profile <= VL53L0X_PROFILE_LONG_RANGE &&
           config->default_i2c_address > 0U &&
           config->default_i2c_address <= 0x7FU &&
           config->target_i2c_address > 0U &&
           config->target_i2c_address <= 0x7FU &&
           config->timeout_ms > 0U;
}

static VL53L0X_Error vl53l0x_apply_profile(VL53L0X *sensor)
{
    VL53L0X_Error error = VL53L0X_ERROR_NONE;
    uint32_t timing_budget_us = sensor->config->timing_budget_us;

    switch (sensor->config->profile) {
        case VL53L0X_PROFILE_HIGH_SPEED:
            if (timing_budget_us == 0U) timing_budget_us = 20000U;
            error = VL53L0X_SetLimitCheckValue(
                &sensor->vendor_device,
                VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                (FixPoint1616_t)16384U);
            if (error == VL53L0X_ERROR_NONE) {
                error = VL53L0X_SetLimitCheckValue(
                    &sensor->vendor_device,
                    VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                    (FixPoint1616_t)(32U * 65536U));
            }
            break;

        case VL53L0X_PROFILE_HIGH_ACCURACY:
            if (timing_budget_us == 0U) timing_budget_us = 200000U;
            error = VL53L0X_SetLimitCheckValue(
                &sensor->vendor_device,
                VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                (FixPoint1616_t)16384U);
            if (error == VL53L0X_ERROR_NONE) {
                error = VL53L0X_SetLimitCheckValue(
                    &sensor->vendor_device,
                    VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                    (FixPoint1616_t)(18U * 65536U));
            }
            break;

        case VL53L0X_PROFILE_LONG_RANGE:
            if (timing_budget_us == 0U) timing_budget_us = 33000U;
            error = VL53L0X_SetLimitCheckValue(
                &sensor->vendor_device,
                VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                (FixPoint1616_t)6554U);
            if (error == VL53L0X_ERROR_NONE) {
                error = VL53L0X_SetLimitCheckValue(
                    &sensor->vendor_device,
                    VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                    (FixPoint1616_t)(60U * 65536U));
            }
            if (error == VL53L0X_ERROR_NONE) {
                error = VL53L0X_SetVcselPulsePeriod(
                    &sensor->vendor_device, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18U);
            }
            if (error == VL53L0X_ERROR_NONE) {
                error = VL53L0X_SetVcselPulsePeriod(
                    &sensor->vendor_device, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14U);
            }
            break;

        case VL53L0X_PROFILE_DEFAULT:
        default:
            if (timing_budget_us == 0U) timing_budget_us = 33000U;
            break;
    }

    if (error == VL53L0X_ERROR_NONE) {
        error = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
            &sensor->vendor_device, timing_budget_us);
    }
    return error;
}

static esp_err_t vl53l0x_enable_hardware(const VL53L0XConfig *config)
{
    if (config->xshut_pin == GPIO_NUM_NC) return ESP_OK;

    gpio_config_t pin_config = {
        .pin_bit_mask = (1ULL << (uint32_t)config->xshut_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t error = gpio_config(&pin_config);
    if (error != ESP_OK) return error;

    error = gpio_set_level(config->xshut_pin, 0);
    if (error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(VL53L0X_BOOT_DELAY_MS));

    error = gpio_set_level(config->xshut_pin, 1);
    if (error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(VL53L0X_BOOT_DELAY_MS));
    return ESP_OK;
}

esp_err_t vl53l0x_init(VL53L0X *sensor, I2cBus *bus,
                       const VL53L0XConfig *config)
{
    if (sensor == NULL || bus == NULL || !vl53l0x_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sensor->initialized || !i2c_bus_is_initialized(bus)) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(sensor, 0, sizeof(*sensor));
    sensor->config = config;
    sensor->bus = bus;

    esp_err_t error = vl53l0x_enable_hardware(config);
    if (error != ESP_OK) goto fail;

    error = i2c_device_init(&sensor->device, bus,
                            config->default_i2c_address);
    if (error != ESP_OK) goto fail;

    sensor->vendor_device.I2cDevAddr = config->default_i2c_address;
    sensor->vendor_device.comms_type = 1U;
    sensor->vendor_device.comms_speed_khz =
        (uint16_t)(bus->config->clock_speed_hz / 1000U);
    sensor->vendor_device.i2c_device = &sensor->device;

    VL53L0X_Error vendor_error = VL53L0X_DataInit(&sensor->vendor_device);
    if (vendor_error != VL53L0X_ERROR_NONE) goto vendor_fail;

    if (config->target_i2c_address != config->default_i2c_address) {
        vendor_error = VL53L0X_SetDeviceAddress(
            &sensor->vendor_device,
            (uint8_t)(config->target_i2c_address << 1U));
        if (vendor_error != VL53L0X_ERROR_NONE) goto vendor_fail;

        vTaskDelay(pdMS_TO_TICKS(VL53L0X_ADDRESS_DELAY_MS));
        sensor->device.address = config->target_i2c_address;
        sensor->vendor_device.I2cDevAddr = config->target_i2c_address;
    }

    VL53L0X_DeviceInfo_t device_info;
    vendor_error = VL53L0X_GetDeviceInfo(&sensor->vendor_device, &device_info);
    if (vendor_error != VL53L0X_ERROR_NONE) goto vendor_fail;
    if (device_info.ProductRevisionMajor != 1U ||
        device_info.ProductRevisionMinor != 1U) {
        error = ESP_ERR_NOT_SUPPORTED;
        goto fail;
    }

    vendor_error = VL53L0X_StaticInit(&sensor->vendor_device);
    if (vendor_error != VL53L0X_ERROR_NONE) goto vendor_fail;

    uint32_t ref_spad_count = 0U;
    uint8_t is_aperture_spads = 0U;
    vendor_error = VL53L0X_PerformRefSpadManagement(
        &sensor->vendor_device, &ref_spad_count, &is_aperture_spads);
    if (vendor_error != VL53L0X_ERROR_NONE) goto vendor_fail;

    uint8_t vhv_settings = 0U;
    uint8_t phase_cal = 0U;
    vendor_error = VL53L0X_PerformRefCalibration(
        &sensor->vendor_device, &vhv_settings, &phase_cal);
    if (vendor_error != VL53L0X_ERROR_NONE) goto vendor_fail;

    vendor_error = vl53l0x_apply_profile(sensor);
    if (vendor_error != VL53L0X_ERROR_NONE) goto vendor_fail;

    sensor->initialized = true;
    return ESP_OK;

vendor_fail:
    error = vl53l0x_convert_error(vendor_error);
fail:
    if (config->xshut_pin != GPIO_NUM_NC) {
        gpio_set_level(config->xshut_pin, 0);
    }
    memset(sensor, 0, sizeof(*sensor));
    return error;
}

esp_err_t vl53l0x_deinit(VL53L0X *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t first_error = ESP_OK;
    if (sensor->ranging) first_error = vl53l0x_stop(sensor);
    if (sensor->config->xshut_pin != GPIO_NUM_NC) {
        esp_err_t gpio_error = gpio_set_level(sensor->config->xshut_pin, 0);
        if (first_error == ESP_OK) first_error = gpio_error;
    }
    memset(sensor, 0, sizeof(*sensor));
    return first_error;
}

esp_err_t vl53l0x_start_continuous(VL53L0X *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized || sensor->ranging) return ESP_ERR_INVALID_STATE;

    VL53L0X_Error error = VL53L0X_SetDeviceMode(
        &sensor->vendor_device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if (error == VL53L0X_ERROR_NONE) {
        error = VL53L0X_StartMeasurement(&sensor->vendor_device);
    }
    if (error != VL53L0X_ERROR_NONE) return vl53l0x_convert_error(error);

    sensor->ranging = true;
    sensor->measurement_valid = false;
    return ESP_OK;
}

esp_err_t vl53l0x_stop(VL53L0X *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized || !sensor->ranging) return ESP_ERR_INVALID_STATE;

    VL53L0X_Error error = VL53L0X_StopMeasurement(&sensor->vendor_device);
    if (error != VL53L0X_ERROR_NONE) return vl53l0x_convert_error(error);

    const int64_t deadline_us = esp_timer_get_time() +
        ((int64_t)sensor->config->timeout_ms * 1000LL);
    uint32_t stop_completed = 0U;
    while (stop_completed == 0U) {
        error = VL53L0X_GetStopCompletedStatus(
            &sensor->vendor_device, &stop_completed);
        if (error != VL53L0X_ERROR_NONE) return vl53l0x_convert_error(error);
        if (esp_timer_get_time() >= deadline_us) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    sensor->ranging = false;
    sensor->measurement_valid = false;
    return ESP_OK;
}

esp_err_t vl53l0x_read_distance(VL53L0X *sensor, uint16_t *distance_mm)
{
    if (sensor == NULL || distance_mm == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized || !sensor->ranging) return ESP_ERR_INVALID_STATE;

    uint8_t ready = 0U;
    VL53L0X_Error error = VL53L0X_GetMeasurementDataReady(
        &sensor->vendor_device, &ready);
    if (error != VL53L0X_ERROR_NONE) return vl53l0x_convert_error(error);
    if (ready == 0U) return ESP_ERR_NOT_FINISHED;

    VL53L0X_RangingMeasurementData_t measurement;
    error = VL53L0X_GetRangingMeasurementData(
        &sensor->vendor_device, &measurement);
    if (error != VL53L0X_ERROR_NONE) return vl53l0x_convert_error(error);

    error = VL53L0X_ClearInterruptMask(&sensor->vendor_device, 0U);
    if (error != VL53L0X_ERROR_NONE) return vl53l0x_convert_error(error);

    sensor->last_distance_mm = measurement.RangeMilliMeter;
    sensor->last_range_status = measurement.RangeStatus;
    sensor->last_update_us = esp_timer_get_time();
    sensor->measurement_valid = measurement.RangeStatus == 0U;
    *distance_mm = sensor->last_distance_mm;
    return ESP_OK;
}

bool vl53l0x_is_measurement_valid(const VL53L0X *sensor)
{
    return sensor != NULL && sensor->initialized && sensor->measurement_valid;
}

uint16_t vl53l0x_get_last_distance_mm(const VL53L0X *sensor)
{
    return sensor != NULL ? sensor->last_distance_mm : 0U;
}
