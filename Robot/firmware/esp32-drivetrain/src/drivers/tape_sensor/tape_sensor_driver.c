/* Implements shared-multiplexer sampling for the three tape sensor modules. */
#include "drivers/tape_sensor/tape_sensor_driver.h"

#include <stddef.h>

#include "esp_rom_sys.h"

// Electrical level that indicates tape detection and mux settling delay.
#define TAPE_SENSOR_ACTIVE_LEVEL 1
#define TAPE_SENSOR_MUX_SETTLE_US 5

static bool mux_config_is_valid(const TapeSensorMuxConfig *config)
{
    return config != NULL &&
           GPIO_IS_VALID_OUTPUT_GPIO(config->channel_select_a_pin) &&
           GPIO_IS_VALID_OUTPUT_GPIO(config->channel_select_b_pin) &&
           config->channel_select_a_pin != config->channel_select_b_pin;
}

static bool sensor_config_is_valid(const TapeSensorDriverConfig *config)
{
    return config != NULL && GPIO_IS_VALID_GPIO(config->module_output_pin);
}

/* Drives mux address A and B. Per the CD4052 truth table,
 * channel = 2*B + A (A is the LSB). */
static void tape_sensor_driver_select_channel(const TapeSensorMux *mux,
                                              TapeSensorChannel channel)
{
    gpio_set_level(mux->config->channel_select_a_pin, channel & 0x01);
    gpio_set_level(mux->config->channel_select_b_pin, (channel >> 1) & 0x01);
    esp_rom_delay_us(TAPE_SENSOR_MUX_SETTLE_US);
}

esp_err_t tape_sensor_mux_init(TapeSensorMux *mux,
                               const TapeSensorMuxConfig *config)
{
    if (mux == NULL || !mux_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (mux->initialized) {
        return mux->config == config ? ESP_OK : ESP_ERR_INVALID_STATE;
    }

    mux->config = config;
    gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << config->channel_select_a_pin) |
                        (1ULL << config->channel_select_b_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t error = gpio_config(&output_config);
    if (error != ESP_OK) {
        return error;
    }

    tape_sensor_driver_select_channel(mux, TAPE_SENSOR_CHANNEL_0);
    mux->initialized = true;
    return ESP_OK;
}

esp_err_t tape_sensor_driver_init(TapeSensor *sensor,
                                  const TapeSensorDriverConfig *config,
                                  TapeSensorMux *mux)
{
    if (sensor == NULL || !sensor_config_is_valid(config) || mux == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!mux->initialized || mux->config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (config->module_output_pin == mux->config->channel_select_a_pin ||
        config->module_output_pin == mux->config->channel_select_b_pin) {
        return ESP_ERR_INVALID_ARG;
    }

    sensor->config = NULL;
    sensor->mux = NULL;
    sensor->channel_0 = false;
    sensor->channel_1 = false;
    sensor->channel_2 = false;
    sensor->channel_3 = false;

    gpio_config_t input_config = {
        .pin_bit_mask = (1ULL << config->module_output_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t error = gpio_config(&input_config);
    if (error != ESP_OK) {
        return error;
    }

    sensor->config = config;
    sensor->mux = mux;
    return ESP_OK;
}

bool tape_sensor_driver_is_on_tape(const TapeSensor *sensor)
{
    if (sensor == NULL || sensor->config == NULL) {
        return false;
    }
    return gpio_get_level(sensor->config->module_output_pin) ==
           TAPE_SENSOR_ACTIVE_LEVEL;
}

esp_err_t tape_sensor_driver_read_all(TapeSensor *sensors[TAPE_SENSOR_MODULE_COUNT])
{
    if (sensors == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int module = 0; module < TAPE_SENSOR_MODULE_COUNT; module++) {
        if (sensors[module] == NULL || sensors[module]->config == NULL ||
            sensors[module]->mux == NULL || !sensors[module]->mux->initialized ||
            sensors[module]->mux->config == NULL ||
            (module > 0 && sensors[module]->mux != sensors[0]->mux)) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    for (int channel = TAPE_SENSOR_CHANNEL_0;
         channel < TAPE_SENSOR_CHANNEL_COUNT;
        channel++) {
        tape_sensor_driver_select_channel(
            sensors[0]->mux, (TapeSensorChannel)channel);

        for (int module = 0; module < TAPE_SENSOR_MODULE_COUNT; module++) {
            bool is_on_tape = tape_sensor_driver_is_on_tape(sensors[module]);
            switch ((TapeSensorChannel)channel) {
                case TAPE_SENSOR_CHANNEL_0:
                    sensors[module]->channel_0 = is_on_tape;
                    break;
                case TAPE_SENSOR_CHANNEL_1:
                    sensors[module]->channel_1 = is_on_tape;
                    break;
                case TAPE_SENSOR_CHANNEL_2:
                    sensors[module]->channel_2 = is_on_tape;
                    break;
                case TAPE_SENSOR_CHANNEL_3:
                    sensors[module]->channel_3 = is_on_tape;
                    break;
                case TAPE_SENSOR_CHANNEL_COUNT:
                    break;
            }
        }
    }
    return ESP_OK;
}

esp_err_t tape_sensor_driver_read_all_raw(
    TapeSensor *sensors[TAPE_SENSOR_MODULE_COUNT],
    uint8_t out_bits[TAPE_SENSOR_MODULE_COUNT])
{
    if (out_bits == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t error = tape_sensor_driver_read_all(sensors);
    if (error != ESP_OK) {
        return error;
    }

    for (int module = 0; module < TAPE_SENSOR_MODULE_COUNT; module++) {
        uint8_t bits = 0;
        bits |= (sensors[module]->channel_0 ? 1U : 0U) << 0;
        bits |= (sensors[module]->channel_1 ? 1U : 0U) << 1;
        bits |= (sensors[module]->channel_2 ? 1U : 0U) << 2;
        bits |= (sensors[module]->channel_3 ? 1U : 0U) << 3;
        out_bits[module] = bits;
    }

    return ESP_OK;
}
