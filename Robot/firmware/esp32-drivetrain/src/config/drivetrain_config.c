#include "config/drivetrain_config.h"

#include "config/motor_config.h"
#include "config/encoder_config.h"

#define DEG_TO_RAD(deg) ((deg) * 3.14159265358979323846f / 180.0f)

#define MAX_DUTY 0.4f
#define WHEEL_ANGLE 30.0f   

const DrivetrainConfig DRIVETRAIN_CONFIG = {
    .motor_configs = {
        &FL_MOTOR_CONFIG, 
        &FR_MOTOR_CONFIG, 
        &BL_MOTOR_CONFIG, 
        &BR_MOTOR_CONFIG
    }, 

    .encoder_configs = {
        &FL_ENCODER_CONFIG, 
        &FR_ENCODER_CONFIG, 
        &BL_ENCODER_CONFIG, 
        &BR_ENCODER_CONFIG
    }, 

    .max_duty = MAX_DUTY, 
    .wheel_angle_rad = DEG_TO_RAD(WHEEL_ANGLE)
};