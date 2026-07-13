#include "config/tape_following_config.h"
#include "sensors/tape_following_PID.h"
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

const TapePidSensorConfig FRONT_PID_WEIGHTS = {
    .weights = { -3, -1, 1, 3 }
};