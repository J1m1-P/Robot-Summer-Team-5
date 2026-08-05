/* Implements ESP32 pulse-counter sampling and comparison-count threshold detection. */
#include "drivers/metal_detector_driver.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "esp_timer.h"

// Constants used for PCNT glitch filtering.
#define METAL_DETECTOR_APB_CLK_HZ 80000000UL
#define METAL_DETECTOR_PCNT_FILTER_MAX_CYCLES 1023

// Converts a nanosecond glitch filter duration to bounded APB clock cycles.
static esp_err_t glitch_ns_to_cycles(
    uint32_t glitch_filter_ns,
    uint16_t *cycles_out) {
    if (cycles_out == NULL) return ESP_ERR_INVALID_ARG;

    if (glitch_filter_ns == 0) {
        *cycles_out = 0;
        return ESP_OK;
    }

    const uint64_t cycles =
        ((uint64_t)glitch_filter_ns * METAL_DETECTOR_APB_CLK_HZ +
         999999999ULL) /
        1000000000ULL;
    if (cycles > METAL_DETECTOR_PCNT_FILTER_MAX_CYCLES) {
        return ESP_ERR_INVALID_ARG;
    }

    *cycles_out = (uint16_t)cycles;
    return ESP_OK;
}

// Configures the pulse-counter channel to count falling edges only.
static esp_err_t configure_channel(const MetalDetectorConfig *config) {
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = config->pulse_pin,
        .ctrl_gpio_num = PCNT_PIN_NOT_USED,
        .channel = config->pcnt_channel,
        .unit = config->pcnt_unit,

        .pos_mode = PCNT_COUNT_DIS,
        .neg_mode = PCNT_COUNT_INC,

        .lctrl_mode = PCNT_MODE_KEEP,
        .hctrl_mode = PCNT_MODE_KEEP,

        .counter_h_lim = config->high_limit,
        .counter_l_lim = config->low_limit,
    };

    return pcnt_unit_config(&pcnt_config);
}

// Checks PCNT resources, pin, limits, sampling, and detection tuning.
bool metal_detector_driver_config_is_valid(const MetalDetectorConfig *config) {
    if (config == NULL) return false;
    if (config->pcnt_unit < 0 || config->pcnt_unit >= PCNT_UNIT_MAX) {
        return false;
    }
    if (config->pcnt_channel < 0 ||
        config->pcnt_channel >= PCNT_CHANNEL_MAX) {
        return false;
    }
    if (!GPIO_IS_VALID_GPIO(config->pulse_pin)) return false;
    if (config->high_limit <= 0 || config->low_limit != 0) return false;
    if (config->sample_period_ms == 0) return false;
    if (!isfinite(config->detect_threshold) ||
        config->detect_threshold <= 0.0f) {
        return false;
    }

    uint16_t filter_cycles = 0;
    return glitch_ns_to_cycles(
        config->glitch_filter_ns, &filter_cycles) == ESP_OK;
}

// Configures the PCNT channel and optional glitch filter for the detector input.
esp_err_t metal_detector_driver_init(
    MetalDetectorDriver *detector,
    const MetalDetectorConfig *config) {
    if (detector == NULL ||
        !metal_detector_driver_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(detector, 0, sizeof(*detector));
    detector->config = config;

    esp_err_t err = configure_channel(config);
    if (err != ESP_OK) return err;

    err = pcnt_counter_pause(config->pcnt_unit);
    if (err != ESP_OK) return err;

    err = pcnt_counter_clear(config->pcnt_unit);
    if (err != ESP_OK) return err;

    if (config->glitch_filter_ns > 0) {
        uint16_t filter_cycles = 0;

        err = glitch_ns_to_cycles(
            config->glitch_filter_ns, &filter_cycles);
        if (err != ESP_OK) return err;

        err = pcnt_set_filter_value(config->pcnt_unit, filter_cycles);
        if (err != ESP_OK) return err;

        err = pcnt_filter_enable(config->pcnt_unit);
        if (err != ESP_OK) return err;
    } else {
        err = pcnt_filter_disable(config->pcnt_unit);
        if (err != ESP_OK) return err;
    }

    detector->initialized = true;
    return ESP_OK;
}

// Clears the hardware counter and resumes counting.
esp_err_t metal_detector_driver_start(MetalDetectorDriver *detector) {
    if (detector == NULL || !detector->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = pcnt_counter_clear(detector->config->pcnt_unit);
    if (err != ESP_OK) return err;

    err = pcnt_counter_resume(detector->config->pcnt_unit);
    if (err != ESP_OK) return err;

    detector->enabled = true;
    detector->sampling = false;
    return ESP_OK;
}

// Pauses the hardware counter and marks the detector disabled.
esp_err_t metal_detector_driver_stop(MetalDetectorDriver *detector) {
    if (detector == NULL || !detector->initialized) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = pcnt_counter_pause(detector->config->pcnt_unit);
    if (err != ESP_OK) return err;

    detector->enabled = false;
    detector->sampling = false;
    return ESP_OK;
}

// Starts a fresh window. Pausing around clear prevents an old partial window
// from leaking into a command-triggered measurement.
esp_err_t metal_detector_driver_begin_sample(MetalDetectorDriver *detector) {
    if (detector == NULL || !detector->initialized || !detector->enabled) {
        return ESP_ERR_INVALID_STATE;
    }
    if (detector->sampling) return ESP_ERR_INVALID_STATE;

    esp_err_t err = pcnt_counter_pause(detector->config->pcnt_unit);
    if (err != ESP_OK) return err;

    err = pcnt_counter_clear(detector->config->pcnt_unit);
    if (err != ESP_OK) {
        (void)pcnt_counter_resume(detector->config->pcnt_unit);
        return err;
    }

    err = pcnt_counter_resume(detector->config->pcnt_unit);
    if (err != ESP_OK) {
        detector->enabled = false;
        return err;
    }

    detector->sample_start_time_us = esp_timer_get_time();
    detector->sampling = true;
    return ESP_OK;
}

// Finishes one window and compares normalized frequency against the baseline.
esp_err_t metal_detector_driver_poll_sample(
    MetalDetectorDriver *detector,
    MetalDetectorSample *sample_out) {

    if (detector == NULL || sample_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!detector->initialized || !detector->enabled || !detector->sampling) {
        return ESP_ERR_INVALID_STATE;
    }

    const int64_t now_us = esp_timer_get_time();
    const int64_t period_us =
        (int64_t)detector->config->sample_period_ms * 1000;
    if (now_us - detector->sample_start_time_us < period_us) {
        return ESP_ERR_NOT_FINISHED;
    }

    esp_err_t err = pcnt_counter_pause(detector->config->pcnt_unit);
    if (err != ESP_OK) return err;

    int16_t raw_count = 0;
    err = pcnt_get_counter_value(detector->config->pcnt_unit, &raw_count);
    const esp_err_t resume_err =
        pcnt_counter_resume(detector->config->pcnt_unit);
    if (resume_err != ESP_OK) detector->enabled = false;
    detector->sampling = false;

    if (err != ESP_OK) return err;
    if (resume_err != ESP_OK) return resume_err;
    if (raw_count <= 0) return ESP_ERR_INVALID_RESPONSE;

    const uint32_t window_us =
        (uint32_t)(now_us - detector->sample_start_time_us);
    const float frequency_hz =
        (float)raw_count * 1000000.0f / (float)window_us;

    MetalDetectorSample sample = {
        .count = raw_count,
        .window_us = window_us,
        .frequency_hz = frequency_hz,
        .baseline_frequency_hz = detector->baseline_frequency_hz,
        .delta_fraction = 0.0f,
        .baseline_valid = detector->baseline_valid,
        .detected = false,
    };

    if (detector->baseline_valid) {
        sample.delta_fraction =
            fabsf(frequency_hz - detector->baseline_frequency_hz) /
            detector->baseline_frequency_hz;
        sample.detected =
            sample.delta_fraction > detector->config->detect_threshold;
    }

    detector->latest_sample = sample;
    *sample_out = sample;
    return ESP_OK;
}

// Captures a completed, live sample as the no-metal reference.
esp_err_t metal_detector_driver_set_baseline(
    MetalDetectorDriver *detector,
    const MetalDetectorSample *sample) {

    if (detector == NULL || sample == NULL) return ESP_ERR_INVALID_ARG;
    if (!detector->initialized || !detector->enabled || detector->sampling) {
        return ESP_ERR_INVALID_STATE;
    }
    if (sample->count <= 0 || !isfinite(sample->frequency_hz) ||
        sample->frequency_hz <= 0.0f) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    detector->baseline_frequency_hz = sample->frequency_hz;
    detector->baseline_valid = true;
    detector->latest_sample = *sample;
    detector->latest_sample.baseline_frequency_hz = sample->frequency_hz;
    detector->latest_sample.delta_fraction = 0.0f;
    detector->latest_sample.baseline_valid = true;
    detector->latest_sample.detected = false;
    return ESP_OK;
}

// Reports whether the detector hardware counter is running.
bool metal_detector_driver_is_enabled(const MetalDetectorDriver *detector) {
    return detector != NULL && detector->initialized && detector->enabled;
}
