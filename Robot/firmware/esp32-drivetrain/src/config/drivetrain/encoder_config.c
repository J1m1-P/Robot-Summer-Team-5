/* Defines quadrature counting and wheel geometry for all four encoders. */
#include "config/pin_map.h"
#include "config/drivetrain/encoder_config.h"

/*
 * Change these based on your real encoder, gearbox, and wheel.
 *
 * Example:
 * encoder PPR = 11
 * gearbox ratio = 30
 * quadrature decoding = 4x
 *
 * counts_per_revolution = 11 * 30 * 4 = 1320
 * 
 * 3200U was got from testing
 */

 #define ENCODER_COUNTS_PER_REVOLUTION 3200U

 /*
 * Example wheel diameter:
 * 65 mm = 0.065 m
 * 
 * 0.070f was got from datasheet
 */

 #define ENCODER_WHEEL_DIAMETER_M 0.070f

 /*
 * Legacy PCNT counter is signed 16-bit.
 * Keep these inside roughly -32768 to 32767.
 */

#define ENCODER_PCNT_HIGH_LIMIT 32000
#define ENCODER_PCNT_LOW_LIMIT  -32000

/*
 * Your driver converts this from ns to APB cycles.
 * 1000 ns = 1 us = about 80 APB cycles at 80 MHz.
 */

 #define ENCODER_GLITCH_FILTER_NS 1000U

 /*
 * Max time to wait for a full quadrature group (4 counts) before falling
 * back to a partial-count velocity update. ~5.5ms is the nominal group
 * period at the 0.05 m/s minimum viable control speed (0.070m wheel,
 * 3200 counts/rev); 10ms adds margin for loop jitter while staying well
 * under DRIVETRAIN_CONFIG.max_control_dt_s (50ms).
 */

 #define ENCODER_LOW_SPEED_TIMEOUT_US 10000U

// Front-left encoder pulse-counter and wheel configuration.
 const EncoderDriverConfig FL_ENCODER_CONFIG = {
    .id = FL_ENCODER, 

    .pcnt_unit = PCNT_UNIT_0, 
    .pcnt_channel_a = PCNT_CHANNEL_0, 
    .pcnt_channel_b = PCNT_CHANNEL_1, 

    .a_pin = PIN_ENC1_A,
    .b_pin = PIN_ENC1_B,

    .direction_inverted = false, 

    .counts_per_revolution = ENCODER_COUNTS_PER_REVOLUTION, 
    .wheel_diameter_m = ENCODER_WHEEL_DIAMETER_M, 

    .high_limit = ENCODER_PCNT_HIGH_LIMIT,
    .low_limit = ENCODER_PCNT_LOW_LIMIT,

    .glitch_filter_ns = ENCODER_GLITCH_FILTER_NS,
    .low_speed_timeout_us = ENCODER_LOW_SPEED_TIMEOUT_US
 };

// Front-right encoder pulse-counter and wheel configuration.
// Physically wired to connector 4 (not 3) -- confirmed against real
// behavior, mirroring the motor connector 2/3 swap already documented in
// motor_config.c's FR_MOTOR_CONFIG comment (encoder connectors 3/4 turned
// out to be swapped independently of that motor swap). direction_inverted
// stays with the physical connector (calibrated against that connector's
// actual count direction), not with the FR label.
  const EncoderDriverConfig FR_ENCODER_CONFIG = {
    .id = FR_ENCODER,

    .pcnt_unit = PCNT_UNIT_1,
    .pcnt_channel_a = PCNT_CHANNEL_0,
    .pcnt_channel_b = PCNT_CHANNEL_1,

    .a_pin = PIN_ENC4_A,
    .b_pin = PIN_ENC4_B,

    .direction_inverted = true,

    .counts_per_revolution = ENCODER_COUNTS_PER_REVOLUTION, 
    .wheel_diameter_m = ENCODER_WHEEL_DIAMETER_M, 

    .high_limit = ENCODER_PCNT_HIGH_LIMIT,
    .low_limit = ENCODER_PCNT_LOW_LIMIT,

    .glitch_filter_ns = ENCODER_GLITCH_FILTER_NS,
    .low_speed_timeout_us = ENCODER_LOW_SPEED_TIMEOUT_US
 };

// Back-left encoder pulse-counter and wheel configuration.
// Physically wired to connector 2 (not 3) -- see FR_ENCODER_CONFIG comment.
  const EncoderDriverConfig BL_ENCODER_CONFIG = {
    .id = BL_ENCODER,

    .pcnt_unit = PCNT_UNIT_2,
    .pcnt_channel_a = PCNT_CHANNEL_0,
    .pcnt_channel_b = PCNT_CHANNEL_1,

    .a_pin = PIN_ENC2_A,
    .b_pin = PIN_ENC2_B,

    .direction_inverted = false,

    .counts_per_revolution = ENCODER_COUNTS_PER_REVOLUTION, 
    .wheel_diameter_m = ENCODER_WHEEL_DIAMETER_M, 

    .high_limit = ENCODER_PCNT_HIGH_LIMIT,
    .low_limit = ENCODER_PCNT_LOW_LIMIT,

    .glitch_filter_ns = ENCODER_GLITCH_FILTER_NS,
    .low_speed_timeout_us = ENCODER_LOW_SPEED_TIMEOUT_US
 };

// Back-right encoder pulse-counter and wheel configuration.
// Physically wired to connector 3 (not 4) -- see FR_ENCODER_CONFIG comment.
  const EncoderDriverConfig BR_ENCODER_CONFIG = {
    .id = BR_ENCODER,

    .pcnt_unit = PCNT_UNIT_3,
    .pcnt_channel_a = PCNT_CHANNEL_0,
    .pcnt_channel_b = PCNT_CHANNEL_1,

    .a_pin = PIN_ENC3_A,
    .b_pin = PIN_ENC3_B,

    .direction_inverted = true,

    .counts_per_revolution = ENCODER_COUNTS_PER_REVOLUTION, 
    .wheel_diameter_m = ENCODER_WHEEL_DIAMETER_M, 

    .high_limit = ENCODER_PCNT_HIGH_LIMIT,
    .low_limit = ENCODER_PCNT_LOW_LIMIT,

    .glitch_filter_ns = ENCODER_GLITCH_FILTER_NS,
    .low_speed_timeout_us = ENCODER_LOW_SPEED_TIMEOUT_US
 };

// Maps EncoderId values to their corresponding hardware configurations.
 const EncoderDriverConfig * const ENCODER_CONFIGS[ENCODER_ID_MAX] = {
    [FL_ENCODER] = &FL_ENCODER_CONFIG, 
    [FR_ENCODER] = &FR_ENCODER_CONFIG, 
    [BL_ENCODER] = &BL_ENCODER_CONFIG, 
    [BR_ENCODER] = &BR_ENCODER_CONFIG
 };
