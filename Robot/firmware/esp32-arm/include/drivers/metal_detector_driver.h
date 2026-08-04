/* Declares pulse-counter sampling and threshold detection for a metal detector oscillator. */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Configuration ------------------------------------------------------------ */

// Defines the pulse-counter channel, pin, and sampling/detection tuning.
typedef struct {
    pcnt_unit_t pcnt_unit;
    pcnt_channel_t pcnt_channel;
    gpio_num_t pulse_pin;

    int16_t high_limit;
    int16_t low_limit;

    uint32_t glitch_filter_ns;

    // Fixed-width window the pulse counter is sampled over. Sets both the
    // detector's update rate and its frequency resolution (1 / window).
    uint32_t sample_period_ms;

    // A frequency change beyond this fraction of the baseline frequency is
    // reported as a detection (e.g. 0.02 = 2%).
    float detect_threshold;
} MetalDetectorConfig;

/* Samples and runtime state --------------------------------------------- */

// One complete, time-normalized detector measurement.
typedef struct {
    int16_t count;
    uint32_t window_us;
    float frequency_hz;
    float baseline_frequency_hz;
    float delta_fraction;
    bool baseline_valid;
    bool detected;
} MetalDetectorSample;

// Holds pulse-counter sampling and baseline state.
typedef struct {
    const MetalDetectorConfig *config;

    int64_t sample_start_time_us;
    float baseline_frequency_hz;
    MetalDetectorSample latest_sample;

    bool initialized;
    bool enabled;
    bool sampling;
    bool baseline_valid;
} MetalDetectorDriver;

/* Lifecycle ----------------------------------------------------------------- */

// Checks whether a detector configuration is safe and internally consistent.
bool metal_detector_driver_config_is_valid(const MetalDetectorConfig *config);

// Configures the ESP32 pulse counter for the detector input.
esp_err_t metal_detector_driver_init(MetalDetectorDriver *detector, const MetalDetectorConfig *config);

// Clears the counter and starts pulse counting.
esp_err_t metal_detector_driver_start(MetalDetectorDriver *detector);

// Pauses pulse counting and marks the detector disabled.
esp_err_t metal_detector_driver_stop(MetalDetectorDriver *detector);

/* Sampling and comparison ----------------------------------------------- */

// Clears the counter and starts a new, independent measurement window.
esp_err_t metal_detector_driver_begin_sample(MetalDetectorDriver *detector);

// Returns ESP_ERR_NOT_FINISHED until the active window has elapsed. A
// successful result is normalized by the actual elapsed time, so loop jitter
// does not appear as a frequency change.
esp_err_t metal_detector_driver_poll_sample(
    MetalDetectorDriver *detector,
    MetalDetectorSample *sample_out);

// Uses a completed, nonzero sample as the no-metal reference.
esp_err_t metal_detector_driver_set_baseline(
    MetalDetectorDriver *detector,
    const MetalDetectorSample *sample);

// Reports whether a valid no-metal reference has been captured.
bool metal_detector_driver_has_baseline(const MetalDetectorDriver *detector);

// Reports whether the detector hardware counter is running.
bool metal_detector_driver_is_enabled(const MetalDetectorDriver *detector);

#ifdef __cplusplus
}
#endif
