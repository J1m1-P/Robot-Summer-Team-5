/* Coordinates configurable ToF startup, shared-bus lifecycle, and polling. */
#include "control/time_of_flight/tof_manager.h"

#include <string.h>

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TOF_SHUTDOWN_DELAY_MS 5U

/* Presents the two driver-specific config arrays as one address topology. */
static const TofDeviceConfig *device_config_at(
    const TofManagerConfig *config, size_t index)
{
    if (index < config->vl53l0x_count) {
        return &config->vl53l0x_configs[index].device;
    }
    return &config->vl53l5cx_configs[index - config->vl53l0x_count].device;
}

/* Ensures storage exists and every shared-bus address conflict is resolvable. */
static bool topology_is_valid(const TofManagerConfig *config,
                              const TofManagerStorage *storage)
{
    if (config == NULL || storage == NULL || config->bus_config == NULL ||
        (config->vl53l0x_count > 0U &&
         (config->vl53l0x_configs == NULL || storage->vl53l0x == NULL)) ||
        (config->vl53l5cx_count > 0U &&
         (config->vl53l5cx_configs == NULL || storage->vl53l5cx == NULL))) {
        return false;
    }

    const size_t count = config->vl53l0x_count + config->vl53l5cx_count;
    if (count == 0U) return false;

    for (size_t first = 0U; first < count; ++first) {
        const TofDeviceConfig *a = device_config_at(config, first);
        if (a->default_i2c_address == 0U || a->default_i2c_address > 0x7FU ||
            a->target_i2c_address == 0U || a->target_i2c_address > 0x7FU) {
            return false;
        }
        for (size_t second = first + 1U; second < count; ++second) {
            const TofDeviceConfig *b = device_config_at(config, second);
            if (a->target_i2c_address == b->target_i2c_address ||
                a->target_i2c_address == b->default_i2c_address ||
                b->target_i2c_address == a->default_i2c_address) {
                return false;
            }
            if (a->shutdown_pin == GPIO_NUM_NC &&
                b->shutdown_pin == GPIO_NUM_NC &&
                a->default_i2c_address == b->default_i2c_address) {
                return false;
            }
        }
    }
    return true;
}

static esp_err_t hold_low(gpio_num_t pin)
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
    return error == ESP_OK ? gpio_set_level(pin, 0U) : error;
}

/* Removes every controllable sensor from the bus before assigning addresses. */
static esp_err_t hold_controllable_sensors(const TofManagerConfig *config)
{
    const size_t count = config->vl53l0x_count + config->vl53l5cx_count;
    for (size_t index = 0U; index < count; ++index) {
        esp_err_t error = hold_low(device_config_at(config, index)->shutdown_pin);
        if (error != ESP_OK) return error;
    }
    vTaskDelay(pdMS_TO_TICKS(TOF_SHUTDOWN_DELAY_MS));
    return ESP_OK;
}

/* Initializes either always-on or shutdown-controlled devices. */
static esp_err_t init_devices(TofManager *manager, bool controlled)
{
    for (size_t index = 0U; index < manager->config->vl53l0x_count; ++index) {
        const VL53L0XConfig *config = &manager->config->vl53l0x_configs[index];
        if ((config->device.shutdown_pin != GPIO_NUM_NC) != controlled) continue;
        esp_err_t error = vl53l0x_driver_init(
            &manager->storage.vl53l0x[index], &manager->bus, config);
        if (error != ESP_OK) return error;
    }
    for (size_t index = 0U; index < manager->config->vl53l5cx_count; ++index) {
        const VL53L5CXConfig *config = &manager->config->vl53l5cx_configs[index];
        if ((config->device.shutdown_pin != GPIO_NUM_NC) != controlled) continue;
        esp_err_t error = vl53l5cx_driver_init(
            &manager->storage.vl53l5cx[index], &manager->bus, config);
        if (error != ESP_OK) return error;
    }
    return ESP_OK;
}

/* Deinitializes one shutdown class in reverse order. */
static esp_err_t deinit_devices(TofManager *manager, bool controlled)
{
    esp_err_t first_error = ESP_OK;
    for (size_t index = manager->config->vl53l5cx_count; index > 0U; --index) {
        VL53L5CX *sensor = &manager->storage.vl53l5cx[index - 1U];
        if (sensor->initialized &&
            ((sensor->config->device.shutdown_pin != GPIO_NUM_NC) == controlled)) {
            esp_err_t error = vl53l5cx_driver_deinit(sensor);
            if (first_error == ESP_OK) first_error = error;
        }
    }
    for (size_t index = manager->config->vl53l0x_count; index > 0U; --index) {
        VL53L0X *sensor = &manager->storage.vl53l0x[index - 1U];
        if (sensor->initialized &&
            ((sensor->config->device.shutdown_pin != GPIO_NUM_NC) == controlled)) {
            esp_err_t error = vl53l0x_driver_deinit(sensor);
            if (first_error == ESP_OK) first_error = error;
        }
    }
    return first_error;
}

/* Powers off controlled devices before always-on devices regain boot addresses. */
static esp_err_t deinit_all_devices(TofManager *manager)
{
    esp_err_t first_error = deinit_devices(manager, true);
    esp_err_t error = deinit_devices(manager, false);
    return first_error == ESP_OK ? error : first_error;
}

/* Stops every active sensor and retains the first error. */
static esp_err_t stop_active_devices(TofManager *manager)
{
    esp_err_t first_error = ESP_OK;
    for (size_t index = 0U; index < manager->config->vl53l5cx_count; ++index) {
        VL53L5CX *sensor = &manager->storage.vl53l5cx[index];
        if (!sensor->ranging) continue;
        esp_err_t error = vl53l5cx_driver_stop(sensor);
        if (first_error == ESP_OK) first_error = error;
    }
    for (size_t index = 0U; index < manager->config->vl53l0x_count; ++index) {
        VL53L0X *sensor = &manager->storage.vl53l0x[index];
        if (!sensor->ranging) continue;
        esp_err_t error = vl53l0x_driver_stop(sensor);
        if (first_error == ESP_OK) first_error = error;
    }
    return first_error;
}

/* Distinguishes a normal not-ready result from a real polling failure. */
static void collect_poll_result(esp_err_t error, bool *updated,
                                esp_err_t *first_error)
{
    if (error == ESP_OK) *updated = true;
    else if (error != ESP_ERR_NOT_FINISHED && *first_error == ESP_OK) {
        *first_error = error;
    }
}

esp_err_t tof_manager_init(TofManager *manager,
                           const TofManagerConfig *config,
                           const TofManagerStorage *storage)
{
    if (manager == NULL || !topology_is_valid(config, storage)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (manager->initialized) return ESP_ERR_INVALID_STATE;

    memset(manager, 0, sizeof(*manager));
    if (config->vl53l0x_count > 0U) {
        memset(storage->vl53l0x, 0,
               config->vl53l0x_count * sizeof(*storage->vl53l0x));
    }
    if (config->vl53l5cx_count > 0U) {
        memset(storage->vl53l5cx, 0,
               config->vl53l5cx_count * sizeof(*storage->vl53l5cx));
    }
    manager->config = config;
    manager->storage = *storage;

    esp_err_t error = hold_controllable_sensors(config);
    if (error != ESP_OK) goto fail_without_bus;
    error = i2c_bus_init(&manager->bus, config->bus_config);
    if (error != ESP_OK) goto fail_without_bus;

    error = init_devices(manager, false);
    if (error == ESP_OK) error = init_devices(manager, true);
    if (error != ESP_OK) goto fail;

    manager->initialized = true;
    return ESP_OK;

fail:
    deinit_all_devices(manager);
    i2c_bus_deinit(&manager->bus);
fail_without_bus:
    memset(manager, 0, sizeof(*manager));
    return error;
}

esp_err_t tof_manager_start(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized || manager->ranging) return ESP_ERR_INVALID_STATE;

    esp_err_t error = ESP_OK;
    for (size_t index = 0U; index < manager->config->vl53l0x_count; ++index) {
        error = vl53l0x_driver_start(&manager->storage.vl53l0x[index]);
        if (error != ESP_OK) goto rollback;
    }
    for (size_t index = 0U; index < manager->config->vl53l5cx_count; ++index) {
        error = vl53l5cx_driver_start(&manager->storage.vl53l5cx[index]);
        if (error != ESP_OK) goto rollback;
    }
    manager->ranging = true;
    return ESP_OK;

rollback:
    stop_active_devices(manager);
    return error;
}

esp_err_t tof_manager_poll(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized || !manager->ranging) return ESP_ERR_INVALID_STATE;

    bool updated = false;
    esp_err_t first_error = ESP_OK;
    for (size_t index = 0U; index < manager->config->vl53l0x_count; ++index) {
        collect_poll_result(
            vl53l0x_driver_read(&manager->storage.vl53l0x[index]),
            &updated, &first_error);
    }
    for (size_t index = 0U; index < manager->config->vl53l5cx_count; ++index) {
        collect_poll_result(
            vl53l5cx_driver_read(&manager->storage.vl53l5cx[index]),
            &updated, &first_error);
    }
    if (first_error != ESP_OK) return first_error;
    return updated ? ESP_OK : ESP_ERR_NOT_FINISHED;
}

esp_err_t tof_manager_stop(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized || !manager->ranging) return ESP_ERR_INVALID_STATE;

    esp_err_t error = stop_active_devices(manager);
    manager->ranging = false;
    return error;
}

esp_err_t tof_manager_deinit(TofManager *manager)
{
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    if (!manager->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t first_error = manager->ranging ? tof_manager_stop(manager) : ESP_OK;
    esp_err_t deinit_error = deinit_all_devices(manager);
    if (first_error == ESP_OK) first_error = deinit_error;
    esp_err_t bus_error = i2c_bus_deinit(&manager->bus);
    if (first_error == ESP_OK) first_error = bus_error;
    memset(manager, 0, sizeof(*manager));
    return first_error;
}
