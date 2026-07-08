#include "config/pin_map.h"
#include "config/encoder_config.h"

/*
 * Change these based on your real encoder, gearbox, and wheel.
 *
 * Example:
 * encoder PPR = 11
 * gearbox ratio = 30
 * quadrature decoding = 4x
 *
 * counts_per_revolution = 11 * 30 * 4 = 1320
 */

 #define ENCODER_COUNTS_PER_REVOLUTION 1320U

 /*
 * Example wheel diameter:
 * 65 mm = 0.065 m
 */

 #define ENCODER_WHEEL_DIAMETER_M 0.065f

 /*
 * Legacy PCNT counter is signed 16-bit.
 * Keep these inside roughly -32768 to 32767.
 */

#define ENCODER_PCNT_HIGH_LIMIT 30000
#define ENCODER_PCNT_LOW_LIMIT  -30000

/*
 * Your driver converts this from ns to APB cycles.
 * 1000 ns = 1 us = about 80 APB cycles at 80 MHz.
 */

 #define ENCODER_GLITCH_FILTER_NS 1000U

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

    .glitch_filter_ns = ENCODER_GLITCH_FILTER_NS
 };

  const EncoderDriverConfig FR_ENCODER_CONFIG = {
    .id = FR_ENCODER, 

    .pcnt_unit = PCNT_UNIT_1, 
    .pcnt_channel_a = PCNT_CHANNEL_0, 
    .pcnt_channel_b = PCNT_CHANNEL_1, 

    .a_pin = PIN_ENC2_A, 
    .b_pin = PIN_ENC2_B, 

    .direction_inverted = false, 

    .counts_per_revolution = ENCODER_COUNTS_PER_REVOLUTION, 
    .wheel_diameter_m = ENCODER_WHEEL_DIAMETER_M, 

    .high_limit = ENCODER_PCNT_HIGH_LIMIT, 
    .low_limit = ENCODER_PCNT_LOW_LIMIT, 

    .glitch_filter_ns = ENCODER_GLITCH_FILTER_NS
 };

  const EncoderDriverConfig BL_ENCODER_CONFIG = {
    .id = BL_ENCODER, 

    .pcnt_unit = PCNT_UNIT_2, 
    .pcnt_channel_a = PCNT_CHANNEL_0, 
    .pcnt_channel_b = PCNT_CHANNEL_1, 

    .a_pin = PIN_ENC3_A, 
    .b_pin = PIN_ENC3_B, 

    .direction_inverted = false, 

    .counts_per_revolution = ENCODER_COUNTS_PER_REVOLUTION, 
    .wheel_diameter_m = ENCODER_WHEEL_DIAMETER_M, 

    .high_limit = ENCODER_PCNT_HIGH_LIMIT, 
    .low_limit = ENCODER_PCNT_LOW_LIMIT, 

    .glitch_filter_ns = ENCODER_GLITCH_FILTER_NS
 };

  const EncoderDriverConfig BR_ENCODER_CONFIG = {
    .id = BR_ENCODER, 

    .pcnt_unit = PCNT_UNIT_3, 
    .pcnt_channel_a = PCNT_CHANNEL_0, 
    .pcnt_channel_b = PCNT_CHANNEL_1, 

    .a_pin = PIN_ENC4_A, 
    .b_pin = PIN_ENC4_B, 

    .direction_inverted = false, 

    .counts_per_revolution = ENCODER_COUNTS_PER_REVOLUTION, 
    .wheel_diameter_m = ENCODER_WHEEL_DIAMETER_M, 

    .high_limit = ENCODER_PCNT_HIGH_LIMIT, 
    .low_limit = ENCODER_PCNT_LOW_LIMIT, 

    .glitch_filter_ns = ENCODER_GLITCH_FILTER_NS
 };

 const EncoderDriverConfig * const ENCODER_CONFIGS[ENCODER_ID_MAX] = {
    [FL_ENCODER] = &FL_ENCODER_CONFIG, 
    [FR_ENCODER] = &FR_ENCODER_CONFIG, 
    [BL_ENCODER] = &BL_ENCODER_CONFIG, 
    [BR_ENCODER] = &BR_ENCODER_CONFIG
 };