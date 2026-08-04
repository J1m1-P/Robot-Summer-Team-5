/* Defines pulse-counter, sampling, and threshold tuning for the metal detector input. */
#include "config/pin_map.h"
#include "config/metal_detector_config.h"

/*
 * Legacy PCNT counter is signed 16-bit. Keep high_limit comfortably above
 * the maximum count expected in one sample window: PCNT resets at this limit,
 * so reaching it aliases the result. low_limit stays at 0 because only
 * falling edges are counted.
 */
#define METAL_DETECTOR_PCNT_HIGH_LIMIT 30000
#define METAL_DETECTOR_PCNT_LOW_LIMIT  0

/*
 * 1000 ns = 1 us = about 80 APB cycles at 80 MHz. Rejects sub-microsecond
 * glitches on the detector line.
 */
#define METAL_DETECTOR_GLITCH_FILTER_NS 1000U

// Sample window: also sets frequency resolution (1 / window = 10 Hz here).
#define METAL_DETECTOR_SAMPLE_PERIOD_MS 100U

// Detection fires when normalized frequency deviates from the sampled
// baseline by more than this fraction.
#define METAL_DETECTOR_DETECT_THRESHOLD 0.015f

const MetalDetectorConfig METAL_DETECTOR_CONFIG = {
    .pcnt_unit = PCNT_UNIT_0,
    .pcnt_channel = PCNT_CHANNEL_0,
    .pulse_pin = (gpio_num_t)PIN_METAL_DETECTOR,

    .high_limit = METAL_DETECTOR_PCNT_HIGH_LIMIT,
    .low_limit = METAL_DETECTOR_PCNT_LOW_LIMIT,

    .glitch_filter_ns = METAL_DETECTOR_GLITCH_FILTER_NS,

    .sample_period_ms = METAL_DETECTOR_SAMPLE_PERIOD_MS,

    .detect_threshold = METAL_DETECTOR_DETECT_THRESHOLD,
};
