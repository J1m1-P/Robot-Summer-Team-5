#include "drivers/encoder_driver.h"

#include <stddef.h>
#include <string.h>

#include "esp_timer.h"

#define ENCODER_PI 3.14159265358979323846f
#define ENCODER_APB_CLK_HZ 80000000UL
#define ENCODER_PCNT_FILTER_MAX_CYCLES 1023

// Helper Functions
static bool encoder_driver_config_is_valid(const EncoderDriverConfig *config) {
    if (config == NULL) return false;
    if (config->id < 0 || config->id >= ENCODER_ID_MAX) return false;
    if (config->a_pin == config->b_pin) return false;
    if (config->pcnt_channel_a == config->pcnt_channel_b) return false;
    if (config->counts_per_revolution == 0) return false;
    if (config->wheel_diameter_m <= 0.0f) return false;
    if (config->high_limit <= 0 || config->low_limit >= 0) return false;
    if (config->low_limit >= config->high_limit) return false;
    return true;
}

static esp_err_t encoder_driver_glitch_ns_to_cycles(uint32_t glitch_filter_ns, uint16_t *cycles_out) {
    if (cycles_out == NULL) return ESP_ERR_INVALID_ARG;

    if (glitch_filter_ns == 0) {
        *cycles_out = 0;
        return ESP_OK;
    }

    uint64_t cycles = ((uint64_t)glitch_filter_ns * ENCODER_APB_CLK_HZ + 999999999ULL) / 1000000000ULL;

    if (cycles == 0) cycles = 1;

    if (cycles > ENCODER_PCNT_FILTER_MAX_CYCLES) return ESP_ERR_INVALID_ARG;

    *cycles_out = (uint16_t)cycles;
    
    return ESP_OK;
}

static esp_err_t encoder_driver_configure_channel_a(const EncoderDriverConfig *config) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = config->a_pin, 
        .ctrl_gpio_num = config->b_pin, 
        .channel = config->pcnt_channel_a, 
        .unit = config->pcnt_unit, 

        .pos_mode = PCNT_COUNT_DEC, 
        .neg_mode = PCNT_COUNT_INC, 

        .lctrl_mode = PCNT_MODE_REVERSE,  
        .hctrl_mode = PCNT_MODE_KEEP, 

        .counter_h_lim = config->high_limit,
        .counter_l_lim = config->low_limit
    };

    return pcnt_unit_config(&pcnt_config);
}

static esp_err_t encoder_driver_configure_channel_b(const EncoderDriverConfig *config) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = config->b_pin, 
        .ctrl_gpio_num = config->a_pin, 
        .channel = config->pcnt_channel_b, 
        .unit = config->pcnt_unit, 

        .pos_mode = PCNT_COUNT_INC, 
        .neg_mode = PCNT_COUNT_DEC, 

        .lctrl_mode = PCNT_MODE_REVERSE,  
        .hctrl_mode = PCNT_MODE_KEEP, 

        .counter_h_lim = config->high_limit,
        .counter_l_lim = config->low_limit
    };

    return pcnt_unit_config(&pcnt_config);
}

// Public Functions
esp_err_t encoder_driver_init(EncoderDriver *encoder, const EncoderDriverConfig *config) {
    if (encoder == NULL || !encoder_driver_config_is_valid(config)) return ESP_ERR_INVALID_ARG;

    memset(encoder, 0, sizeof(*encoder));
    encoder->config = *config;

    esp_err_t err;

    err = encoder_driver_configure_channel_a(config);
    if (err != ESP_OK) return err;

    err = encoder_driver_configure_channel_b(config);
    if (err != ESP_OK) return err;

    err = pcnt_counter_pause(config->pcnt_unit);
    if (err != ESP_OK) return err;

    err = pcnt_counter_clear(config->pcnt_unit);
    if (err != ESP_OK) return err;

    if (config->glitch_filter_ns > 0) {
        uint16_t filter_cycles = 0;

        err = encoder_driver_glitch_ns_to_cycles(config->glitch_filter_ns, &filter_cycles);
        if (err != ESP_OK) return err;

        err = pcnt_set_filter_value(config->pcnt_unit, filter_cycles);
        if (err != ESP_OK) return err;

        err = pcnt_filter_enable(config->pcnt_unit);
        if (err != ESP_OK) return err;
    }
    else {
        err = pcnt_filter_disable(config->pcnt_unit);
        if (err != ESP_OK) return err;
    }

    encoder->last_timestamp_us = esp_timer_get_time();
    encoder->initialized = true;

    return ESP_OK;
}

esp_err_t encoder_driver_start(EncoderDriver *encoder) {
    if (encoder == NULL || !encoder->initialized) return ESP_ERR_INVALID_ARG;

    int32_t current_count = 0;
    esp_err_t err = encoder_driver_get_count(encoder, &current_count);
    if (err != ESP_OK) return err;

    err = pcnt_counter_resume(encoder->config.pcnt_unit);
    if (err != ESP_OK) return err;

    encoder->enabled = true;
    encoder->last_count = current_count;
    encoder->last_timestamp_us = esp_timer_get_time();

    return ESP_OK;
}

esp_err_t encoder_driver_stop(EncoderDriver *encoder) {
    if (encoder == NULL || !encoder->initialized) return ESP_ERR_INVALID_ARG;

    esp_err_t err = pcnt_counter_pause(encoder->config.pcnt_unit);
    if (err != ESP_OK) return err;

    encoder->enabled = false;

    return ESP_OK;
}

esp_err_t encoder_driver_reset(EncoderDriver *encoder) {
    if (encoder == NULL || !encoder->initialized) return ESP_ERR_INVALID_ARG;

    esp_err_t err;
    err = pcnt_counter_pause(encoder->config.pcnt_unit); 
    if (err != ESP_OK) return err;
    err = pcnt_counter_clear(encoder->config.pcnt_unit);
    if (err != ESP_OK) return err;

    encoder->last_count = 0;
    encoder->last_timestamp_us = esp_timer_get_time();
    encoder->velocity_mps = 0.0f;
    encoder->velocity_rps = 0.0f;

    if (encoder->enabled) {
        err = pcnt_counter_resume(encoder->config.pcnt_unit);
        if (err != ESP_OK) {
            encoder->enabled = false;
            return err;
        }
    }

    return ESP_OK;
}

esp_err_t encoder_driver_get_count(const EncoderDriver *encoder, int32_t *count) {
    if (encoder == NULL || !encoder->initialized || count == NULL) return ESP_ERR_INVALID_ARG;

    int16_t raw_count = 0;

    esp_err_t err = pcnt_get_counter_value(encoder->config.pcnt_unit, &raw_count);
    if (err != ESP_OK) return err;

    int32_t adjusted_count = (int32_t)raw_count;
    if (encoder->config.direction_inverted) adjusted_count = -adjusted_count;

    *count = adjusted_count;

    return ESP_OK;
}

esp_err_t encoder_driver_get_revolutions(const EncoderDriver *encoder, float *revolutions) {
    if (encoder == NULL || revolutions == NULL) return ESP_ERR_INVALID_ARG;

    int32_t count = 0;

    esp_err_t err = encoder_driver_get_count(encoder, &count);
    if (err != ESP_OK) return err;

    *revolutions = (float)count / (float)encoder->config.counts_per_revolution;

    return ESP_OK;
}

esp_err_t encoder_driver_get_distance_m(const EncoderDriver *encoder, float *distance_m) {
    if (encoder == NULL || distance_m == NULL) return ESP_ERR_INVALID_ARG;

    float revolutions = 0.0f;
    
    esp_err_t err = encoder_driver_get_revolutions(encoder, &revolutions);
    if (err != ESP_OK) return err;

    float circumference_m = ENCODER_PI * encoder->config.wheel_diameter_m;

    *distance_m = revolutions * circumference_m;

    return ESP_OK;
}

esp_err_t encoder_driver_update_velocity(EncoderDriver *encoder) {
    if (encoder == NULL || !encoder->initialized) return ESP_ERR_INVALID_ARG;

    int32_t current_count = 0;

    esp_err_t err = encoder_driver_get_count(encoder, &current_count);
    if (err != ESP_OK) return err;

    int64_t current_timestamp_us = esp_timer_get_time();

    int32_t delta_count = current_count - encoder->last_count;
    int64_t delta_time_us = current_timestamp_us - encoder->last_timestamp_us;

    if (delta_time_us <= 0) return ESP_ERR_INVALID_STATE;

    float delta_time_s = (float)delta_time_us / 1000000.0f;
    float delta_revolutions = (float)delta_count / (float)encoder->config.counts_per_revolution;
    float circumference_m = ENCODER_PI * encoder->config.wheel_diameter_m;

    encoder->velocity_rps = delta_revolutions / delta_time_s;
    encoder->velocity_mps = circumference_m * encoder->velocity_rps;

    encoder->last_count = current_count;
    encoder->last_timestamp_us = current_timestamp_us;

    return ESP_OK;
}

float encoder_driver_get_velocity_rps(const EncoderDriver *encoder) {
    if (encoder == NULL || !encoder->initialized) return 0.0f;

    return encoder->velocity_rps;
}

float encoder_driver_get_velocity_mps(const EncoderDriver *encoder) {
    if (encoder == NULL || !encoder->initialized) return 0.0f;

    return encoder->velocity_mps;
}

bool encoder_driver_is_initialized(const EncoderDriver *encoder) {
    if (encoder == NULL) return false;

    return encoder->initialized;
}

bool encoder_driver_is_enabled(const EncoderDriver *encoder) {
    if (encoder == NULL || !encoder->initialized) return false;

    return encoder->enabled;
}