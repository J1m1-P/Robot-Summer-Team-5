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

    // abs(count - comparison_count) beyond this fraction of abs(comparison_count)
    // is reported as a detection (e.g. 0.02 = 2%).
    float detect_threshold;
} MetalDetectorConfig;

/* Runtime state --------------------------------------------------------- */

// Holds pulse-counter scheduling and the comparison count/result.
typedef struct {
    const MetalDetectorConfig *config;

    int64_t next_sample_time_us;

    int16_t count;
    int16_t comparison_count;
    bool detected;

    bool initialized;
    bool enabled;
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

/* Comparison ------------------------------------------------------------ */

// Sets the pulse count that subsequent reads are compared against.
void metal_detector_driver_set_comparison_count(MetalDetectorDriver *detector, int16_t comparison_count);

// Samples the counter once its window elapses (returns the cached result
// otherwise) and reports whether it deviates from the comparison count by
// more than detect_threshold as a fraction of the comparison count.
bool metal_detector_driver_read(MetalDetectorDriver *detector);

// Returns the most recently sampled raw pulse count (0 if never sampled).
int16_t metal_detector_driver_get_count(const MetalDetectorDriver *detector);

// Reports whether the detector hardware counter is running.
bool metal_detector_driver_is_enabled(const MetalDetectorDriver *detector);

#ifdef __cplusplus
}
#endif
