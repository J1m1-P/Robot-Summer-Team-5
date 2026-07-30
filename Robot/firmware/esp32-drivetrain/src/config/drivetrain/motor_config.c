/* Defines PWM and direction settings for all four drivetrain motors. */
#include "config/pin_map.h"
#include "config/drivetrain/motor_config.h"

// Shared PWM timing and maximum duty settings for every drivetrain motor.
#define MOTOR_PWM_FREQUENCY     9000       // 9kHz
#define MOTOR_PWM_RESOLUTION    12           // 2**12 resolution
#define MOTOR_MAX_DUTY          0.95f       // 95% duty cycle

// Front-left motor pin, PWM channel, and direction configuration.
const MotorDriverConfig FL_MOTOR_CONFIG = {
    .pwm_pin = PIN_M1_PWM,
    .dir_pin = PIN_M1_DIR,

    .pwm_channel = 0,
    .pwm_frequency = MOTOR_PWM_FREQUENCY,
    .pwm_resolution = MOTOR_PWM_RESOLUTION,

    .direction_inverted = true,
    
    .max_duty = MOTOR_MAX_DUTY
};

// Front-right motor pin, PWM channel, and direction configuration.
// Physically wired to connector 3 (not 2) -- see the wheel-numbering fix in
// misc/algo_testing.py: 1=FL, 2=BL, 3=FR, 4=BR, confirmed against real
// pure-rotation behavior (wheels 1&2 spin opposite from 3&4, i.e. left
// side vs right side), which only holds if connector 3 is front-right.
const MotorDriverConfig FR_MOTOR_CONFIG = {
    .pwm_pin = PIN_M3_PWM,
    .dir_pin = PIN_M3_DIR,

    .pwm_channel = 1,
    .pwm_frequency = MOTOR_PWM_FREQUENCY,
    .pwm_resolution = MOTOR_PWM_RESOLUTION,

    .direction_inverted = true,
    
    .max_duty = MOTOR_MAX_DUTY
};

// Back-left motor pin, PWM channel, and direction configuration.
// Physically wired to connector 2 (not 3) -- see the FR_MOTOR_CONFIG
// comment above; connectors 2 and 3 were swapped from the 1=FL/2=FR/3=BL/4=BR
// labeling this file originally used.
const MotorDriverConfig BL_MOTOR_CONFIG = {
    .pwm_pin = PIN_M2_PWM,
    .dir_pin = PIN_M2_DIR,

    .pwm_channel = 2,
    .pwm_frequency = MOTOR_PWM_FREQUENCY,
    .pwm_resolution = MOTOR_PWM_RESOLUTION,

    .direction_inverted = true,
    
    .max_duty = MOTOR_MAX_DUTY
};

// Back-right motor pin, PWM channel, and direction configuration.
const MotorDriverConfig BR_MOTOR_CONFIG = {
    .pwm_pin = PIN_M4_PWM,
    .dir_pin = PIN_M4_DIR,

    .pwm_channel = 3,
    .pwm_frequency = MOTOR_PWM_FREQUENCY,
    .pwm_resolution = MOTOR_PWM_RESOLUTION,

    .direction_inverted = true,

    .max_duty = MOTOR_MAX_DUTY
};
