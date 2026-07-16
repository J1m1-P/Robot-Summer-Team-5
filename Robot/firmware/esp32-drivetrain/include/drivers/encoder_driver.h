/* Declares quadrature encoder counting, distance tracking, and velocity estimation. */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "driver/pcnt.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif


// Identifies each physical wheel encoder and bounds encoder lookup arrays.
typedef enum {
    FL_ENCODER = 0,
    FR_ENCODER,
    BL_ENCODER,
    BR_ENCODER, 
    ENCODER_ID_MAX
} EncoderId;

// Defines pulse-counter channels, pins, wheel geometry, and input filtering.
typedef struct {
    EncoderId id;

    pcnt_unit_t pcnt_unit;
    pcnt_channel_t pcnt_channel_a;
    pcnt_channel_t pcnt_channel_b;

    gpio_num_t a_pin;
    gpio_num_t b_pin;

    bool direction_inverted;

    uint32_t counts_per_revolution;
    float wheel_diameter_m;

    int16_t high_limit;
    int16_t low_limit;

    uint32_t glitch_filter_ns;
} EncoderDriverConfig;

// Holds accumulated count, velocity estimates, and encoder lifecycle state.
typedef struct {
    const EncoderDriverConfig *config;

    int32_t accumulated_count;
    int64_t last_timestamp_us;

    float velocity_mps;
    float velocity_rps;

    bool initialized;
    bool enabled;
} EncoderDriver;

// Checks whether an encoder configuration is safe and internally consistent.
bool encoder_driver_config_is_valid(const EncoderDriverConfig *config);

// Configures the ESP32 pulse counter for one quadrature encoder.
esp_err_t encoder_driver_init(EncoderDriver *encoder, const EncoderDriverConfig *config);

// Clears and starts pulse counting for an initialized encoder.
esp_err_t encoder_driver_start(EncoderDriver *encoder);

// Pauses pulse counting and marks the encoder disabled.
esp_err_t encoder_driver_stop(EncoderDriver *encoder);

// Clears hardware and accumulated counts plus velocity history.
esp_err_t encoder_driver_reset(EncoderDriver *encoder);

// Returns the total count including accumulated hardware counter deltas.
esp_err_t encoder_driver_get_count(const EncoderDriver *encoder, int32_t *count);

// Converts the total count into wheel revolutions.
esp_err_t encoder_driver_get_revolutions(const EncoderDriver *encoder, float *revolutions);

// Converts the total count into linear wheel travel in meters.
esp_err_t encoder_driver_get_distance_m(const EncoderDriver *encoder, float *distance_m);

// Samples the hardware counter and updates accumulated count and velocity.
esp_err_t encoder_driver_update(EncoderDriver *encoder);

// Reports whether the encoder driver has been initialized.
bool encoder_driver_is_initialized(const EncoderDriver *encoder);

// Reports whether pulse counting is currently enabled.
bool encoder_driver_is_enabled(const EncoderDriver *encoder);

// Returns the latest angular velocity estimate in revolutions per second.
float encoder_driver_get_velocity_rps(const EncoderDriver *encoder);

// Returns the latest linear velocity estimate in meters per second.
float encoder_driver_get_velocity_mps(const EncoderDriver *encoder);


#ifdef __cplusplus
}
#endif
