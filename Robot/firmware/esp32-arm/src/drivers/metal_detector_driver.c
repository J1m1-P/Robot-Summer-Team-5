/* Implements ESP32 pulse-counter sampling and comparison-count threshold detection. */
#include "drivers/metal_detector_driver.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "esp_timer.h"

// Constants used for PCNT glitch filtering.
#define METAL_DETECTOR_APB_CLK_HZ 80000000UL
#define METAL_DETECTOR_PCNT_FILTER_MAX_CYCLES 1023

// Checks pcnt resources, pin, limits, sampling, and detection tuning.
bool metal_detector_driver_config_is_valid(const MetalDetectorConfig *config) {
    if (config == NULL) return false;
    if (config->pcnt_unit < 0 || config->pcnt_unit >= PCNT_UNIT_MAX) return false;
    if (config->pcnt_channel < 0 || config->pcnt_channel >= PCNT_CHANNEL_MAX) return false;
    if (!GPIO_IS_VALID_GPIO(config->pulse_pin)) return false;
    if (config->high_limit <= 0 || config->low_limit != 0) return false;
    if (config->sample_period_ms == 0) return false;
    if (!isfinite(config->detect_threshold) || config->detect_threshold <= 0.0f) return false;
    uint64_t filter_cycles =
        ((uint64_t)config->glitch_filter_ns * METAL_DETECTOR_APB_CLK_HZ + 999999999ULL) /
        1000000000ULL;
    if (filter_cycles > METAL_DETECTOR_PCNT_FILTER_MAX_CYCLES) return false;
    return true;
}

// Converts a nanosecond glitch filter duration to bounded APB clock cycles.
static esp_err_t metal_detector_driver_glitch_ns_to_cycles(uint32_t glitch_filter_ns, uint16_t *cycles_out) {
    if (cycles_out == NULL) return ESP_ERR_INVALID_ARG;

    if (glitch_filter_ns == 0) {
        *cycles_out = 0;
        return ESP_OK;
    }

    uint64_t cycles = ((uint64_t)glitch_filter_ns * METAL_DETECTOR_APB_CLK_HZ + 999999999ULL) / 1000000000ULL;

    if (cycles == 0) cycles = 1;

    if (cycles > METAL_DETECTOR_PCNT_FILTER_MAX_CYCLES) return ESP_ERR_INVALID_ARG;

    *cycles_out = (uint16_t)cycles;

    return ESP_OK;
}

// Configures the pulse-counter channel to count rising edges only.
static esp_err_t metal_detector_driver_configure_channel(const MetalDetectorConfig *config) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = config->pulse_pin,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .channel = config->pcnt_channel,
        .unit = config->pcnt_unit,

        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,

        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,

        .counter_h_lim = config->high_limit,
        .counter_l_lim = config->low_limit
    };

    return pcnt_unit_config(&pcnt_config);
}

// Configures the PCNT channel and optional glitch filter for the detector input.
esp_err_t metal_detector_driver_init(MetalDetectorDriver *detector, const MetalDetectorConfig *config) {
    if (detector == NULL || !metal_detector_driver_config_is_valid(config)) return ESP_ERR_INVALID_ARG;

    memset(detector, 0, sizeof(*detector));
    detector->config = config;

    esp_err_t err = metal_detector_driver_configure_channel(config);
    if (err != ESP_OK) return err;

    err = pcnt_counter_pause(config->pcnt_unit);
    if (err != ESP_OK) return err;

    err = pcnt_counter_clear(config->pcnt_unit);
    if (err != ESP_OK) return err;

    if (config->glitch_filter_ns > 0) {
        uint16_t filter_cycles = 0;

        err = metal_detector_driver_glitch_ns_to_cycles(config->glitch_filter_ns, &filter_cycles);
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

    detector->initialized = true;

    return ESP_OK;
}

// Clears the hardware counter and resumes counting.
esp_err_t metal_detector_driver_start(MetalDetectorDriver *detector) {
    if (detector == NULL || !detector->initialized) return ESP_ERR_INVALID_ARG;

    esp_err_t err = pcnt_counter_clear(detector->config->pcnt_unit);
    if (err != ESP_OK) return err;

    err = pcnt_counter_resume(detector->config->pcnt_unit);
    if (err != ESP_OK) return err;

    detector->enabled = true;
    detector->next_sample_time_us =
        esp_timer_get_time() + (int64_t)detector->config->sample_period_ms * 1000;

    return ESP_OK;
}

// Pauses the hardware counter and marks the detector disabled.
esp_err_t metal_detector_driver_stop(MetalDetectorDriver *detector) {
    if (detector == NULL || !detector->initialized) return ESP_ERR_INVALID_ARG;

    esp_err_t err = pcnt_counter_pause(detector->config->pcnt_unit);
    if (err != ESP_OK) return err;

    detector->enabled = false;

    return ESP_OK;
}

// Sets the pulse count that subsequent reads are compared against.
void metal_detector_driver_set_comparison_count(MetalDetectorDriver *detector, int16_t comparison_count) {
    if (detector == NULL) return;

    detector->comparison_count = comparison_count;
}

// Samples the counter once its window elapses and compares it to
// comparison_count; returns the cached result if the window hasn't closed.
bool metal_detector_driver_read(MetalDetectorDriver *detector) {
    if (detector == NULL || !detector->initialized || !detector->enabled) return false;

    int64_t now = esp_timer_get_time();
    if (now < detector->next_sample_time_us) return detector->detected;

    int64_t period_us = (int64_t)detector->config->sample_period_ms * 1000;
    detector->next_sample_time_us = (now - detector->next_sample_time_us >= period_us)
        ? now + period_us
        : detector->next_sample_time_us + period_us;

    int16_t raw_count = 0;
    if (pcnt_get_counter_value(detector->config->pcnt_unit, &raw_count) != ESP_OK) {
        return detector->detected;
    }
    if (pcnt_counter_clear(detector->config->pcnt_unit) != ESP_OK) {
        return detector->detected;
    }

    detector->count = raw_count;

    float delta = (float)raw_count - (float)detector->comparison_count;
    float threshold = fabsf((float)detector->comparison_count) * detector->config->detect_threshold;
    detector->detected = fabsf(delta) > threshold;

    return detector->detected;
}

// Returns the most recently sampled raw pulse count.
int16_t metal_detector_driver_get_count(const MetalDetectorDriver *detector) {
    if (detector == NULL || !detector->initialized) return 0;

    return detector->count;
}

// Reports whether the detector hardware counter is running.
bool metal_detector_driver_is_enabled(const MetalDetectorDriver *detector) {
    if (detector == NULL || !detector->initialized) return false;

    return detector->enabled;
}
