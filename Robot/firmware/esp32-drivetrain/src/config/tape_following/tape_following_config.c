/* Defines tape-sensor hardware configuration. */
#include "config/tape_following/tape_following_config.h"

#include "config/pin_map.h"

/* Shared multiplexer and tape-module hardware assignments. */
const TapeSensorMuxConfig TAPE_SENSOR_MUX_CONFIG = {
    .channel_select_a_pin = PIN_TF_CHSEL1_PIN,
    .channel_select_b_pin = PIN_TF_CHSEL2_PIN,
};

const TapeSensorDriverConfig FRONT_TAPE_SENSOR_CONFIG = {
    .module_output_pin = PIN_TF_FRONT_INPUT,
};

const TapeSensorDriverConfig BACK_TAPE_SENSOR_CONFIG = {
    .module_output_pin = PIN_TF_BACK_INPUT,
};

const TapeSensorDriverConfig LEFT_TAPE_SENSOR_CONFIG = {
    .module_output_pin = PIN_TF_LEFT_INPUT,
};
