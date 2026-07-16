/* Defines the complete four-wheel drivetrain hardware and motion limits. */
#include "config/drivetrain_config.h"

#include "config/pin_map.h"
#include "config/motor_config.h"
#include "config/encoder_config.h"

// Converts a compile-time wheel angle from degrees to radians.
#define DEG_TO_RAD(deg) ((deg) * 3.14159265358979323846f / 180.0f)

// Limits wheel commands to 40 percent duty during initial testing.
#define MAX_DUTY 0.4f

// Physical X-drive wheel-force angle measured from the body axes.
#define WHEEL_ANGLE 30.0f   

// Connects all motor and encoder configurations with brake and geometry settings.
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
    .wheel_angle_rad = DEG_TO_RAD(WHEEL_ANGLE), 

    .brk_pin = PIN_M_BRK
};
