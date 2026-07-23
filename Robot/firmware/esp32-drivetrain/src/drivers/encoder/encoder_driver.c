/* Implements ESP32 pulse-counter quadrature tracking and velocity estimation. */
#include "drivers/encoder/encoder_driver.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "esp_timer.h"

// Constants used for wheel-distance conversion and PCNT glitch filtering.
#define ENCODER_PI 3.14159265358979323846f
#define ENCODER_APB_CLK_HZ 80000000UL
#define ENCODER_PCNT_FILTER_MAX_CYCLES 1023

// Counts per encoder line under 4x quadrature decoding (fixed by how channels
// A and B are configured below, not tunable). Velocity is only computed over
// whole multiples of this many counts so that the uneven sub-line spacing
// between A-rise/B-rise/A-fall/B-fall cancels out instead of showing up as
// ripple. Note: if direction reverses mid-group, the carried remainder no
// longer represents a coherent group of one direction; the arithmetic stays
// sign-safe, but the cancellation rationale only strictly holds within a
// single direction of rotation.
#define ENCODER_QUADRATURE_GROUP_SIZE 4

// Checks encoder IDs, pins, PCNT resources, geometry, limits, and filter range.
bool encoder_driver_config_is_valid(const EncoderDriverConfig *config) {
    if (config == NULL) return false;
    if (config->id < 0 || config->id >= ENCODER_ID_MAX) return false;
    if (config->pcnt_unit < 0 || config->pcnt_unit >= PCNT_UNIT_MAX) return false;
    if (config->pcnt_channel_a < 0 || config->pcnt_channel_a >= PCNT_CHANNEL_MAX) return false;
    if (config->pcnt_channel_b < 0 || config->pcnt_channel_b >= PCNT_CHANNEL_MAX) return false;
    if (!GPIO_IS_VALID_GPIO(config->a_pin) || !GPIO_IS_VALID_GPIO(config->b_pin)) return false;
    if (config->a_pin == config->b_pin) return false;
    if (config->pcnt_channel_a == config->pcnt_channel_b) return false;
    if (config->counts_per_revolution == 0) return false;
    if (!isfinite(config->wheel_diameter_m) || config->wheel_diameter_m <= 0.0f) return false;
    if (config->high_limit <= 0 || config->low_limit >= 0) return false;
    if (config->low_limit >= config->high_limit) return false;
    if (config->low_speed_timeout_us == 0) return false;
    uint64_t filter_cycles =
        ((uint64_t)config->glitch_filter_ns * ENCODER_APB_CLK_HZ + 999999999ULL) /
        1000000000ULL;
    if (filter_cycles > ENCODER_PCNT_FILTER_MAX_CYCLES) return false;
    return true;
}

// Converts a nanosecond glitch filter duration to bounded APB clock cycles.
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

// Configures channel A for full quadrature decoding using channel B as control.
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

// Configures channel B for full quadrature decoding using channel A as control.
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

// Reads the signed hardware counter and applies configured direction inversion.
static esp_err_t encoder_driver_read_raw_delta(const EncoderDriver *encoder, int32_t *delta_count) {
    if (encoder == NULL || delta_count == NULL) return ESP_ERR_INVALID_ARG;

    int16_t raw_count = 0;

    esp_err_t err = pcnt_get_counter_value(encoder->config->pcnt_unit, &raw_count);
    if (err != ESP_OK) return err;

    int32_t adjusted_count = (int32_t)raw_count;

    if (encoder->config->direction_inverted) adjusted_count = -adjusted_count;

    *delta_count = adjusted_count;

    return ESP_OK;
}

// Atomically transfers the hardware counter delta into the accumulated count.
static esp_err_t encoder_driver_flush_delta(EncoderDriver *encoder, int32_t *delta_count) {
    if (encoder == NULL || delta_count == NULL) return ESP_ERR_INVALID_ARG;

    pcnt_unit_t unit = encoder->config->pcnt_unit;

    esp_err_t err;
    bool enabled = encoder->enabled;

    if (enabled) {
        err = pcnt_counter_pause(unit);
        if (err != ESP_OK) {
            return err;
        }
    }

    int32_t delta = 0;

    err = encoder_driver_read_raw_delta(encoder, &delta);
    if (err != ESP_OK) {
        if (enabled) {
            pcnt_counter_resume(unit);
        }
        return err;
    }

    err = pcnt_counter_clear(unit);
    if (err != ESP_OK) {
        if (enabled) {
            pcnt_counter_resume(unit);
        }
        return err;
    }

    if (enabled) {
        err = pcnt_counter_resume(unit);
        if (err != ESP_OK) {
            encoder->enabled = false;
            return err;
        }
    }

    encoder->accumulated_count += delta;
    *delta_count = delta;

    return ESP_OK;
}

// Configures both PCNT channels and the optional glitch filter for one encoder.
esp_err_t encoder_driver_init(EncoderDriver *encoder, const EncoderDriverConfig *config) {
    if (encoder == NULL || !encoder_driver_config_is_valid(config)) return ESP_ERR_INVALID_ARG;

    memset(encoder, 0, sizeof(*encoder));
    encoder->config = config;

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

    int64_t now = esp_timer_get_time();
    encoder->last_timestamp_us = now;
    encoder->velocity_window_start_us = now;
    encoder->initialized = true;

    return ESP_OK;
}

// Clears and resumes the hardware counter for an initialized encoder.
esp_err_t encoder_driver_start(EncoderDriver *encoder) {
    if (encoder == NULL || !encoder->initialized) return ESP_ERR_INVALID_ARG;

    esp_err_t err = pcnt_counter_resume(encoder->config->pcnt_unit);
    if (err != ESP_OK) return err;

    encoder->enabled = true;

    int64_t now = esp_timer_get_time();
    encoder->last_timestamp_us = now;
    encoder->velocity_window_start_us = now;
    encoder->velocity_remainder_count = 0;

    return ESP_OK;
}

// Pauses the hardware counter and marks the encoder disabled.
esp_err_t encoder_driver_stop(EncoderDriver *encoder) {
    if (encoder == NULL || !encoder->initialized) return ESP_ERR_INVALID_ARG;

    esp_err_t err = pcnt_counter_pause(encoder->config->pcnt_unit);
    if (err != ESP_OK) return err;

    encoder->enabled = false;

    return ESP_OK;
}

// Clears hardware count, accumulated count, timestamps, and velocity estimates.
esp_err_t encoder_driver_reset(EncoderDriver *encoder) {
    if (encoder == NULL || !encoder->initialized) return ESP_ERR_INVALID_ARG;

    bool was_enabled = encoder->enabled;

    esp_err_t err;
    err = pcnt_counter_pause(encoder->config->pcnt_unit); 
    if (err != ESP_OK) return err;
    err = pcnt_counter_clear(encoder->config->pcnt_unit);
    if (err != ESP_OK) return err;

    encoder->accumulated_count = 0;

    int64_t now = esp_timer_get_time();
    encoder->last_timestamp_us = now;
    encoder->velocity_window_start_us = now;
    encoder->velocity_remainder_count = 0;

    encoder->velocity_mps = 0.0f;
    encoder->velocity_rps = 0.0f;

    if (was_enabled) {
        err = pcnt_counter_resume(encoder->config->pcnt_unit);
        if (err != ESP_OK) {
            encoder->enabled = false;
            return err;
        }
    }

    encoder->enabled = was_enabled;

    return ESP_OK;
}

// Returns accumulated count plus the current unflushed hardware delta.
esp_err_t encoder_driver_get_count(const EncoderDriver *encoder, int32_t *count) {
    if (encoder == NULL || !encoder->initialized || count == NULL) return ESP_ERR_INVALID_ARG;

    int32_t pending_delta = 0;

    esp_err_t err = encoder_driver_read_raw_delta(encoder, &pending_delta);
    if (err != ESP_OK) return err;


    *count = encoder->accumulated_count + pending_delta;

    return ESP_OK;
}

// Converts the current total count into wheel revolutions.
esp_err_t encoder_driver_get_revolutions(const EncoderDriver *encoder, float *revolutions) {
    if (encoder == NULL || revolutions == NULL) return ESP_ERR_INVALID_ARG;

    int32_t count = 0;

    esp_err_t err = encoder_driver_get_count(encoder, &count);
    if (err != ESP_OK) return err;

    *revolutions = (float)count / (float)encoder->config->counts_per_revolution;

    return ESP_OK;
}

// Converts wheel revolutions into signed linear travel in meters.
esp_err_t encoder_driver_get_distance_m(const EncoderDriver *encoder, float *distance_m) {
    if (encoder == NULL || distance_m == NULL) return ESP_ERR_INVALID_ARG;

    float revolutions = 0.0f;
    
    esp_err_t err = encoder_driver_get_revolutions(encoder, &revolutions);
    if (err != ESP_OK) return err;

    float circumference_m = ENCODER_PI * encoder->config->wheel_diameter_m;

    *distance_m = revolutions * circumference_m;

    return ESP_OK;
}

// Flushes the latest delta and updates angular and linear velocity estimates.
esp_err_t encoder_driver_update(EncoderDriver *encoder) {
    if (encoder == NULL || !encoder->initialized) return ESP_ERR_INVALID_ARG;

    int64_t current_timestamp_us = esp_timer_get_time();
    int64_t delta_time_us = current_timestamp_us - encoder->last_timestamp_us;
    if (delta_time_us <= 0) return ESP_ERR_INVALID_STATE;

    int32_t delta_count = 0;
    esp_err_t err = encoder_driver_flush_delta(encoder, &delta_count);
    if (err != ESP_OK) return err;

    // Only compute velocity over whole quadrature groups (see
    // ENCODER_QUADRATURE_GROUP_SIZE) so per-edge asymmetry cancels out; any
    // leftover counts are carried into the next window. If a full group
    // hasn't formed within low_speed_timeout_us, fall back to whatever
    // partial count is pending so low-speed readings stay responsive and
    // decay to true zero on an actual stop.
    int32_t total_pending = encoder->velocity_remainder_count + delta_count;
    int32_t usable = total_pending - (total_pending % ENCODER_QUADRATURE_GROUP_SIZE);
    bool timed_out = (current_timestamp_us - encoder->velocity_window_start_us) >=
                      (int64_t)encoder->config->low_speed_timeout_us;

    if (usable == 0 && !timed_out) {
        // Not enough counts yet and still within the timeout: hold the last
        // velocity estimate rather than reporting a false zero.
        encoder->velocity_remainder_count = total_pending;
        encoder->last_timestamp_us = current_timestamp_us;
        return ESP_OK;
    }

    int32_t counts_for_velocity = (usable != 0) ? usable : total_pending;
    int64_t window_us = current_timestamp_us - encoder->velocity_window_start_us;
    float window_s = (float)window_us / 1000000.0f;

    float delta_revolutions = (float)counts_for_velocity / (float)encoder->config->counts_per_revolution;
    float circumference_m = ENCODER_PI * encoder->config->wheel_diameter_m;

    encoder->velocity_rps = delta_revolutions / window_s;
    encoder->velocity_mps = circumference_m * encoder->velocity_rps;

    encoder->velocity_remainder_count = total_pending - counts_for_velocity;
    encoder->velocity_window_start_us = current_timestamp_us;
    encoder->last_timestamp_us = current_timestamp_us;

    return ESP_OK;
}

// Returns the latest angular velocity estimate or zero when unavailable.
float encoder_driver_get_velocity_rps(const EncoderDriver *encoder) {
    if (encoder == NULL || !encoder->initialized) return 0.0f;

    return encoder->velocity_rps;
}

// Returns the latest linear velocity estimate or zero when unavailable.
float encoder_driver_get_velocity_mps(const EncoderDriver *encoder) {
    if (encoder == NULL || !encoder->initialized) return 0.0f;

    return encoder->velocity_mps;
}

// Reports whether the encoder completed initialization.
bool encoder_driver_is_initialized(const EncoderDriver *encoder) {
    if (encoder == NULL) return false;

    return encoder->initialized;
}

// Reports whether the encoder hardware counter is running.
bool encoder_driver_is_enabled(const EncoderDriver *encoder) {
    if (encoder == NULL || !encoder->initialized) return false;

    return encoder->enabled;
}
