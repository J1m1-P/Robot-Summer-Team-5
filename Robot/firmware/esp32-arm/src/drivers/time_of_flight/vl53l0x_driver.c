/* Implements lifecycle and cached single-zone ranging for VL53L0X sensors. */
#include "drivers/time_of_flight/vl53l0x_driver.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define VL53L0X_BOOT_DELAY_MS     5U
#define VL53L0X_ADDRESS_DELAY_MS 10U

/* -------------------------------------------------------------------------- */
/* Test-only diagnostics                                                       */
/* -------------------------------------------------------------------------- */

#if defined(TOF_ENABLE_TEST_DIAGNOSTICS)
extern void vl53l0x_test_diagnostic_event(const VL53L0XConfig *config,
                                          const char *state, uint8_t address,
                                          esp_err_t error);

static void test_report_state(const VL53L0XConfig *config, const char *state,
                              uint8_t address, esp_err_t error)
{
    vl53l0x_test_diagnostic_event(config, state, address, error);
}
#else
#define test_report_state(config, state, address, error) ((void)0)
#endif

/* -------------------------------------------------------------------------- */
/* Validation and vendor error conversion                                      */
/* -------------------------------------------------------------------------- */

static esp_err_t convert_error(VL53L0X_Error error)
{
    switch (error) {
        case VL53L0X_ERROR_NONE: return ESP_OK;
        case VL53L0X_ERROR_INVALID_PARAMS: return ESP_ERR_INVALID_ARG;
        case VL53L0X_ERROR_TIME_OUT: return ESP_ERR_TIMEOUT;
        case VL53L0X_ERROR_NOT_SUPPORTED:
        case VL53L0X_ERROR_MODE_NOT_SUPPORTED: return ESP_ERR_NOT_SUPPORTED;
        default: return ESP_FAIL;
    }
}

static bool config_is_valid(const VL53L0XConfig *config)
{
    return config != NULL &&
           config->profile >= VL53L0X_PROFILE_DEFAULT &&
           config->profile <= VL53L0X_PROFILE_LONG_RANGE &&
           config->device.default_i2c_address > 0U &&
           config->device.default_i2c_address <= 0x7FU &&
           config->device.target_i2c_address > 0U &&
           config->device.target_i2c_address <= 0x7FU &&
           (config->timing_budget_us == 0U ||
            config->timing_budget_us >= 20000U) &&
           config->stop_timeout_ms > 0U && config->stale_after_ms > 0U;
}

/* -------------------------------------------------------------------------- */
/* Power and address management                                                */
/* -------------------------------------------------------------------------- */

/* Finds an always-on device after an ESP reset, when it may retain its target address. */
static esp_err_t find_initial_address(I2cBus *bus,
                                     const TofDeviceConfig *config,
                                     uint8_t *address)
{
    *address = config->default_i2c_address;
    if (config->shutdown_pin != GPIO_NUM_NC ||
        config->target_i2c_address == config->default_i2c_address) {
        return ESP_OK;
    }
    esp_err_t default_error = i2c_bus_probe(bus, config->default_i2c_address);
    if (default_error == ESP_OK) return ESP_OK;
    esp_err_t target_error = i2c_bus_probe(bus, config->target_i2c_address);
    if (target_error == ESP_OK) {
        *address = config->target_i2c_address;
        return ESP_OK;
    }
    return default_error;
}

/* Updates the hardware, project transport, and ST context together. */
static VL53L0X_Error set_address(VL53L0X *sensor, uint8_t address)
{
    VL53L0X_Error error = VL53L0X_SetDeviceAddress(
        &sensor->vendor_device, (uint8_t)(address << 1U));
    if (error == VL53L0X_ERROR_NONE) {
        vTaskDelay(pdMS_TO_TICKS(VL53L0X_ADDRESS_DELAY_MS));
        sensor->i2c_device.address = address;
        sensor->vendor_device.I2cDevAddr = address;
    }
    return error;
}

/* Resets and boots a sensor when its XSHUT signal is controlled. */
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
    if (error == ESP_OK) error = gpio_set_level(config->shutdown_pin, 0U);
    if (error != ESP_OK) return error;
    vTaskDelay(pdMS_TO_TICKS(VL53L0X_BOOT_DELAY_MS));

    error = gpio_set_level(config->shutdown_pin, 1U);
    if (error == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(VL53L0X_BOOT_DELAY_MS));
    }
    return error;
}

/* -------------------------------------------------------------------------- */
/* Sensor configuration                                                        */
/* -------------------------------------------------------------------------- */

/* Applies the selected profile and its explicit or default timing budget. */
static VL53L0X_Error apply_profile(VL53L0X *sensor)
{
    uint32_t budget_us = sensor->config->timing_budget_us;
    FixPoint1616_t signal_limit = 0U;
    FixPoint1616_t sigma_limit = 0U;
    bool long_range = false;

    switch (sensor->config->profile) {
        case VL53L0X_PROFILE_HIGH_SPEED:
            if (budget_us == 0U) budget_us = 20000U;
            signal_limit = (FixPoint1616_t)16384U;
            sigma_limit = (FixPoint1616_t)(32U * 65536U);
            break;
        case VL53L0X_PROFILE_HIGH_ACCURACY:
            if (budget_us == 0U) budget_us = 200000U;
            signal_limit = (FixPoint1616_t)16384U;
            sigma_limit = (FixPoint1616_t)(18U * 65536U);
            break;
        case VL53L0X_PROFILE_LONG_RANGE:
            if (budget_us == 0U) budget_us = 33000U;
            signal_limit = (FixPoint1616_t)6554U;
            sigma_limit = (FixPoint1616_t)(60U * 65536U);
            long_range = true;
            break;
        case VL53L0X_PROFILE_DEFAULT:
        default:
            if (budget_us == 0U) budget_us = 33000U;
            break;
    }

    VL53L0X_Error error = VL53L0X_ERROR_NONE;
    if (signal_limit != 0U) {
        error = VL53L0X_SetLimitCheckValue(
            &sensor->vendor_device,
            VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signal_limit);
    }
    if (error == VL53L0X_ERROR_NONE && sigma_limit != 0U) {
        error = VL53L0X_SetLimitCheckValue(
            &sensor->vendor_device,
            VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigma_limit);
    }
    if (error == VL53L0X_ERROR_NONE && long_range) {
        error = VL53L0X_SetVcselPulsePeriod(
            &sensor->vendor_device, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18U);
    }
    if (error == VL53L0X_ERROR_NONE && long_range) {
        error = VL53L0X_SetVcselPulsePeriod(
            &sensor->vendor_device, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14U);
    }
    if (error == VL53L0X_ERROR_NONE) {
        error = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(
            &sensor->vendor_device, budget_us);
    }
    return error;
}

/* -------------------------------------------------------------------------- */
/* Public lifecycle API                                                        */
/* -------------------------------------------------------------------------- */

esp_err_t vl53l0x_driver_init(VL53L0X *sensor, I2cBus *bus,
                              const VL53L0XConfig *config)
{
    if (sensor == NULL || bus == NULL || !config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (sensor->initialized || !i2c_bus_is_initialized(bus)) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(sensor, 0, sizeof(*sensor));
    sensor->config = config;
    test_report_state(config, "powering_on",
                      config->device.default_i2c_address, ESP_OK);
    esp_err_t error = power_on(&config->device);
    if (error != ESP_OK) goto fail;
    uint8_t initial_address = config->device.default_i2c_address;
    error = find_initial_address(bus, &config->device, &initial_address);
    if (error != ESP_OK) goto fail;
    test_report_state(config, "boot_address", initial_address, ESP_OK);
    error = i2c_device_init(&sensor->i2c_device, bus, initial_address);
    if (error != ESP_OK) goto fail;

    sensor->vendor_device.I2cDevAddr = initial_address;
    sensor->vendor_device.comms_type = 1U;
    sensor->vendor_device.comms_speed_khz =
        (uint16_t)(bus->config->clock_speed_hz / 1000U);
    sensor->vendor_device.i2c_device = &sensor->i2c_device;

    VL53L0X_Error vendor_error = VL53L0X_DataInit(&sensor->vendor_device);
    if (vendor_error != VL53L0X_ERROR_NONE) goto vendor_fail;
    test_report_state(config, "identity_verified", initial_address, ESP_OK);
    if (config->device.target_i2c_address != initial_address) {
        test_report_state(config, "changing_address", initial_address, ESP_OK);
        vendor_error = set_address(sensor, config->device.target_i2c_address);
        if (vendor_error != VL53L0X_ERROR_NONE) goto vendor_fail;
    }
    test_report_state(config, "address_assigned",
                      sensor->i2c_device.address, ESP_OK);
    test_report_state(config, "calibrating",
                      sensor->i2c_device.address, ESP_OK);

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

    uint32_t spad_count;
    uint8_t aperture_spads;
    vendor_error = VL53L0X_PerformRefSpadManagement(
        &sensor->vendor_device, &spad_count, &aperture_spads);
    if (vendor_error != VL53L0X_ERROR_NONE) goto vendor_fail;

    uint8_t vhv_settings;
    uint8_t phase_calibration;
    vendor_error = VL53L0X_PerformRefCalibration(
        &sensor->vendor_device, &vhv_settings, &phase_calibration);
    if (vendor_error != VL53L0X_ERROR_NONE) goto vendor_fail;

    vendor_error = apply_profile(sensor);
    if (vendor_error != VL53L0X_ERROR_NONE) goto vendor_fail;

    sensor->initialized = true;
    test_report_state(config, "ready", sensor->i2c_device.address, ESP_OK);
    return ESP_OK;

vendor_fail:
    error = convert_error(vendor_error);
fail:
    test_report_state(
        config, "failed",
        sensor->i2c_device.initialized
            ? sensor->i2c_device.address
            : config->device.default_i2c_address,
        error);
    if (config->device.shutdown_pin != GPIO_NUM_NC) {
        gpio_set_level(config->device.shutdown_pin, 0U);
    } else if (sensor->i2c_device.initialized &&
               sensor->i2c_device.address != config->device.default_i2c_address) {
        set_address(sensor, config->device.default_i2c_address);
    }
    memset(sensor, 0, sizeof(*sensor));
    return error;
}

esp_err_t vl53l0x_driver_deinit(VL53L0X *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t first_error = sensor->ranging
        ? vl53l0x_driver_stop(sensor) : ESP_OK;
    if (sensor->config->device.shutdown_pin != GPIO_NUM_NC) {
        esp_err_t pin_error = gpio_set_level(
            sensor->config->device.shutdown_pin, 0U);
        if (first_error == ESP_OK) first_error = pin_error;
    } else if (sensor->config->device.target_i2c_address !=
               sensor->config->device.default_i2c_address) {
        VL53L0X_Error address_error = set_address(
            sensor, sensor->config->device.default_i2c_address);
        if (first_error == ESP_OK) first_error = convert_error(address_error);
    }
    memset(sensor, 0, sizeof(*sensor));
    return first_error;
}

esp_err_t vl53l0x_driver_start(VL53L0X *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized || sensor->ranging) return ESP_ERR_INVALID_STATE;

    VL53L0X_Error error = VL53L0X_SetDeviceMode(
        &sensor->vendor_device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);
    if (error == VL53L0X_ERROR_NONE) {
        error = VL53L0X_StartMeasurement(&sensor->vendor_device);
    }
    if (error != VL53L0X_ERROR_NONE) return convert_error(error);

    sensor->has_data = false;
    sensor->ranging = true;
    test_report_state(sensor->config, "ranging",
                      sensor->i2c_device.address, ESP_OK);
    return ESP_OK;
}

esp_err_t vl53l0x_driver_stop(VL53L0X *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized || !sensor->ranging) return ESP_ERR_INVALID_STATE;

    VL53L0X_Error error = VL53L0X_StopMeasurement(&sensor->vendor_device);
    if (error != VL53L0X_ERROR_NONE) return convert_error(error);

    const int64_t deadline_us = esp_timer_get_time() +
        (int64_t)sensor->config->stop_timeout_ms * 1000LL;
    uint32_t stopped = 0U;
    while (stopped == 0U) {
        error = VL53L0X_GetStopCompletedStatus(
            &sensor->vendor_device, &stopped);
        if (error != VL53L0X_ERROR_NONE) return convert_error(error);
        if (esp_timer_get_time() >= deadline_us) return ESP_ERR_TIMEOUT;
        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    sensor->has_data = false;
    sensor->ranging = false;
    return ESP_OK;
}

/* -------------------------------------------------------------------------- */
/* Public data API                                                             */
/* -------------------------------------------------------------------------- */

esp_err_t vl53l0x_driver_read(VL53L0X *sensor)
{
    if (sensor == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized || !sensor->ranging) return ESP_ERR_INVALID_STATE;

    uint8_t ready = 0U;
    VL53L0X_Error error = VL53L0X_GetMeasurementDataReady(
        &sensor->vendor_device, &ready);
    if (error != VL53L0X_ERROR_NONE) return convert_error(error);
    if (ready == 0U) return ESP_ERR_NOT_FINISHED;

    VL53L0X_RangingMeasurementData_t measurement;
    error = VL53L0X_GetRangingMeasurementData(
        &sensor->vendor_device, &measurement);
    if (error == VL53L0X_ERROR_NONE) {
        error = VL53L0X_ClearInterruptMask(&sensor->vendor_device, 0U);
    }
    if (error != VL53L0X_ERROR_NONE) return convert_error(error);

    sensor->sample.distance_mm = measurement.RangeMilliMeter;
    sensor->sample.range_status = measurement.RangeStatus;
    sensor->sample.timestamp_us = esp_timer_get_time();
    sensor->sample.valid = measurement.RangeStatus == 0U;
    sensor->has_data = true;
    return ESP_OK;
}

esp_err_t vl53l0x_driver_get_sample(const VL53L0X *sensor,
                                    VL53L0XSample *sample)
{
    if (sensor == NULL || sample == NULL) return ESP_ERR_INVALID_ARG;
    if (!sensor->initialized || !sensor->has_data ||
        esp_timer_get_time() - sensor->sample.timestamp_us >
            (int64_t)sensor->config->stale_after_ms * 1000LL) {
        return ESP_ERR_INVALID_STATE;
    }
    *sample = sensor->sample;
    return ESP_OK;
}
