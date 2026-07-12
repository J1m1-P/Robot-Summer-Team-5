#include "sensors/tape_following.h"
#include "config/pin_map.h"
#include <stddef.h>
#include "esp_rom_sys.h" /* esp_rom_delay_us */

#define TAPE_ACTIVE_LEVEL 1
#define TAPE_MUX_SETTLE_US 5

static bool s_chsel_initialized = false;

/* Drive TF_CHSEL1 (mux address A) and TF_CHSEL2 (mux address B).
 * Per the CD4052 truth table, channel = 2*B + A (A is the LSB). 
 * All 3 modules share the same mux, so this is a single global function.
 */
static void tape_sensor_select_channel(TapeChannel channel)
{
    gpio_set_level(PIN_TF_CHSEL1_PIN, channel & 0x01);
    gpio_set_level(PIN_TF_CHSEL2_PIN, (channel >> 1) & 0x01);
    esp_rom_delay_us(TAPE_MUX_SETTLE_US); // Allow time for the mux to settle before reading the outputs
}


esp_err_t tape_sensor_init(TapeSensor *sensor, const TapeSensorConfig *config)
{
    if (sensor == NULL || config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    sensor->config = config; // Store the config pointer for later use
    sensor->sensor_0 = false; // Initialize sensor readings to false
    sensor->sensor_1 = false;
    sensor->sensor_2 = false;
    sensor->sensor_3 = false;

    // Configure the module_out pin as an input
    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << config->module_out),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&in_cfg);
    if (err != ESP_OK) {
        return err;
    }

    // Configure the shared mux select pins as outputs (only once)
    if (!s_chsel_initialized) {
        gpio_config_t out_cfg = {
            .pin_bit_mask = (1ULL << PIN_TF_CHSEL1_PIN) | (1ULL << PIN_TF_CHSEL2_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        err = gpio_config(&out_cfg);
        if (err != ESP_OK) {
            return err;
        }
        tape_sensor_select_channel(TAPE_CH_0);
        s_chsel_initialized = true;
    }

    return ESP_OK;
}


bool pin_is_on_tape(TapeSensor *sensor)
{
    if (sensor == NULL || sensor->config == NULL) {
        return false;
    }
    return gpio_get_level(sensor->config->module_out) == TAPE_ACTIVE_LEVEL; 
}

// Array of 3 pointers to the 3 sensor modules (front, back, left)
esp_err_t tape_sensor_read_all_channels(TapeSensor *sensors[TAPE_MODULE_COUNT])
{
    if (sensors == NULL) { 
        return ESP_ERR_INVALID_ARG;
    }
    for (int m = 0; m < TAPE_MODULE_COUNT; m++) {
        if (sensors[m] == NULL || sensors[m]->config == NULL) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    // Cycle through all 4 channels
    for (int ch = TAPE_CH_0; ch <= TAPE_CH_3; ch++) {
        tape_sensor_select_channel((TapeChannel)ch);

        // Read all 3 modules for this channel
        for (int m = 0; m < TAPE_MODULE_COUNT; m++) {
            bool on_tape = pin_is_on_tape(sensors[m]);
            switch ((TapeChannel)ch) { 
                case TAPE_CH_0: sensors[m]->sensor_0 = on_tape; break;
                case TAPE_CH_1: sensors[m]->sensor_1 = on_tape; break;
                case TAPE_CH_2: sensors[m]->sensor_2 = on_tape; break;
                case TAPE_CH_3: sensors[m]->sensor_3 = on_tape; break;
            }
        }
    }
    return ESP_OK;
}

esp_err_t tape_sensor_read_all_channels_raw(TapeSensor *sensors[TAPE_MODULE_COUNT], uint8_t out_bits[TAPE_MODULE_COUNT])
{
    if (out_bits == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = tape_sensor_read_all_channels(sensors);
    if (err != ESP_OK) {
        return err;
    }
    // out_bits[m] is an array of 3 bytes, one for each module. Each byte is the 4 bit module reading condensed into one number.
    for (int m = 0; m < TAPE_MODULE_COUNT; m++) {
        uint8_t bits = 0;
        bits |= (sensors[m]->sensor_0 ? 1 : 0) << 0;
        bits |= (sensors[m]->sensor_1 ? 1 : 0) << 1;
        bits |= (sensors[m]->sensor_2 ? 1 : 0) << 2;
        bits |= (sensors[m]->sensor_3 ? 1 : 0) << 3;
        out_bits[m] = bits;
    }

    return ESP_OK;
}