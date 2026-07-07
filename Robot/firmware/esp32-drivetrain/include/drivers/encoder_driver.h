// #pragma once 

// #include <stdint.h>
// #include <stdbool.h>

// #include "driver/gpio.h"
// #include "driver/pcnt.h"
// #include "esp_err.h"

// typedef enum {
//     FL_ENCODER = 0,
//     FR_ENCODER,
//     BL_ENCODER,
//     BR_ENCODER, 
//     ENCODER_ID_MAX
// } EncoderId;

// typedef struct {
//     EncoderId id;

//     pcnt_unit_t pcnt_unit;
//     pcnt_channel_t pcnt_channel_a;
//     pcnt_channel_t pcnt_channel_b;

//     gpio_num_t a_pin;
//     gpio_num_t b_pin;

//     bool direction_inverted;

//     uint32_t counts_per_revolution;
//     float wheel_diameter_m;

//     int16_t high_limit;
//     int16_t low_limit;

//     uint32_t glitch_filter_ns;
// } EncoderDriverConfig;

// typedef struct {
//     EncoderDriverConfig config;

//     int32_t last_count;
//     int64_t last_timestamp_us;

//     float velocity_mps;
//     float velocity_rps;

//     bool initialized;
//     bool enabled;
// } EncoderDriver;

// // Initialization
// esp_err_t encoder_driver_init(EncoderDriver *encoder, const EncoderDriverConfig *config);

// // Counter Control
// esp_err_t encoder_driver_start(EncoderDriver *encoder);
// esp_err_t encoder_driver_stop(EncoderDriver *encoder);
// esp_err_t encoder_driver_reset(EncoderDriver *encoder);

// esp_err_t encoder_driver_get_count(EncoderDriver *encoder, int32_t *count);
// esp_err_t encoder_driver_get_revolution(EncoderDriver *encoder, float *revolutions);
// esp_err_t encoder_driver_get_distance_m(EncoderDriver *encoder, float *distance_m);

// esp_err_t encoder_driver_update_velocity(EncoderDriver *encoder);

// // Status
// bool encoder_driver_is_initialized(const EncoderDriver *encoder);
// bool encoder_driver_is_enabled(const EncoderDriver *encoder);
// float encoder_driver_get_velocity_rps(EncoderDriver *encoder);
// float encoder_driver_get_velocity_mps(EncoderDriver *encoder);