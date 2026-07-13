#include "config/tape_following_config.h"
#include "config/pin_map.h"

const TapeSensorConfig TAPE_SENSOR_FRONT = {
    .module_out = PIN_TF1_INPUT,
};

const TapeSensorConfig TAPE_SENSOR_BACK = {
    .module_out = PIN_TF2_INPUT,
};

const TapeSensorConfig TAPE_SENSOR_LEFT = {
    .module_out = PIN_TF3_INPUT,
};