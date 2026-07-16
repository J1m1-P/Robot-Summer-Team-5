/* Defines tape sensor inputs and their physical channel-position weights. */
#include "config/tape_following_config.h"
#include "sensors/tape_following_PID.h"
#include "config/pin_map.h"

// Front tape sensor module input configuration.
const TapeSensorConfig TAPE_SENSOR_FRONT = {
    .module_out = PIN_TF1_INPUT,
};

// Back tape sensor module input configuration.
const TapeSensorConfig TAPE_SENSOR_BACK = {
    .module_out = PIN_TF2_INPUT,
};

// Left tape sensor module input configuration.
const TapeSensorConfig TAPE_SENSOR_LEFT = {
    .module_out = PIN_TF3_INPUT,
};

// Front module channel weights ordered from left to right.
const TapePidSensorConfig FRONT_PID_WEIGHTS = {
    .weights = { -3, -1, 1, 3 }
};

// Back module channel weights ordered from left to right.
const TapePidSensorConfig BACK_PID_WEIGHTS = {
    .weights = { -3, -1, 1, 3 }
};

// Left module channel weights ordered from left to right.
const TapePidSensorConfig LEFT_PID_WEIGHTS = {
    .weights = { -3, -1, 1, 3 }
};
