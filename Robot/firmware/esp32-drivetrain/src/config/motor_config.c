#include "config/pin_map.h"
#include "config/motor_config.h"

// Change these
#define MOTOR_PWM_FREQUENCY     20000       // 20kHz
#define MOTOR_PWM_RESOLURION    8           // 2**8 resolution
#define MOTOR_MAX_DUTY          0.60f       // 60% duty cycle

const MotorDriverConfig FL_MOTOR_CONFIG = {
    .pwm_pin = PIN_M1_PWM, 
    .dir_pin = PIN_M1_DIR, 

    .pwm_channel = 0, 
    .pwm_frequency = MOTOR_PWM_FREQUENCY, 
    .pwm_resolution = MOTOR_PWM_RESOLURION, 

    .direction_inverted = false, 
    
    .max_duty = MOTOR_MAX_DUTY
};

const MotorDriverConfig FR_MOTOR_CONFIG = {
    .pwm_pin = PIN_M2_PWM, 
    .dir_pin = PIN_M2_DIR, 

    .pwm_channel = 1, 
    .pwm_frequency = MOTOR_PWM_FREQUENCY, 
    .pwm_resolution = MOTOR_PWM_RESOLURION, 

    .direction_inverted = false, 
    
    .max_duty = MOTOR_MAX_DUTY
};

const MotorDriverConfig BL_MOTOR_CONFIG = {
    .pwm_pin = PIN_M3_PWM, 
    .dir_pin = PIN_M3_DIR, 

    .pwm_channel = 2, 
    .pwm_frequency = MOTOR_PWM_FREQUENCY, 
    .pwm_resolution = MOTOR_PWM_RESOLURION, 

    .direction_inverted = false, 
    
    .max_duty = MOTOR_MAX_DUTY
};

const MotorDriverConfig BR_MOTOR_CONFIG = {
    .pwm_pin = PIN_M4_PWM, 
    .dir_pin = PIN_M4_DIR, 

    .pwm_channel = 3, 
    .pwm_frequency = MOTOR_PWM_FREQUENCY, 
    .pwm_resolution = MOTOR_PWM_RESOLURION, 

    .direction_inverted = false, 
    
    .max_duty = MOTOR_MAX_DUTY 
};