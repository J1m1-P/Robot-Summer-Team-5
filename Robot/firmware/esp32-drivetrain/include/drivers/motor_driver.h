#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Motor Driver Configuration Structure (Compile time data)
typedef struct {
    uint8_t pwm_pin;            // Pin for PWM control
    uint8_t dir_pin;            // Pin for direction control

    uint8_t pwm_channel;        // PWM channel for motor control
    uint32_t pwm_frequency;     // PWM frequency for motor control (Hz)
    uint8_t pwm_resolution;     // PWM resolution (bits)

    bool direction_inverted;    // Flag to indicate if the motor direction is inverted

    float max_duty;             // Maximum duty cycle for the motor (0.0 to 1.0)
} MotorDriverConfig;


// Motor Driver Structure (Run time data)
typedef struct {
    const MotorDriverConfig *config;   // Configuration for the motor driver

    bool initialized;                   // Flag to indicate if the motor driver is initialized
    bool enabled;                       // Flag to indicate if the motor driver is enabled
    bool coasting;                      // Flag to indicate if the motor is currently coasting

    float current_duty;                 // Current duty cycle for the motor (-max_duty to +max_duty)
} MotorDriver;

// Initialization
bool motor_driver_config_is_valid(const MotorDriverConfig *config);
esp_err_t motor_driver_init(MotorDriver *motor, const MotorDriverConfig *config);
esp_err_t motor_driver_enable(MotorDriver *motor);
esp_err_t motor_driver_disable(MotorDriver *motor);

// Motor Control
esp_err_t motor_driver_set_duty(MotorDriver *motor, float duty);         // Set the motor duty cycle (-max_duty to +max_duty), determines speed 
esp_err_t motor_driver_coast(MotorDriver *motor);                        // Set the motor pwm to 0 and let it coast

// Status 
bool motor_driver_is_initialized(const MotorDriver *motor);     // Check if the motor driver is initialized
bool motor_driver_is_enabled(const MotorDriver *motor);         // Check if the motor driver is enabled
bool motor_driver_is_coasting(const MotorDriver *motor);        // Check if the motor is currently coasting
float motor_driver_get_current_duty(const MotorDriver *motor);  // Get the current duty cycle of the motor


#ifdef __cplusplus
}
#endif
