/* Declares PWM motor setup, duty control, enablement, and coasting state. */
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

// Checks whether a motor configuration is safe and internally consistent.
bool motor_driver_config_is_valid(const MotorDriverConfig *config);

// Configures the motor's direction GPIO and LEDC PWM channel.
esp_err_t motor_driver_init(MotorDriver *motor, const MotorDriverConfig *config);

// Enables duty commands for an initialized motor.
esp_err_t motor_driver_enable(MotorDriver *motor);

// Sets duty to zero and rejects further commands until re-enabled.
esp_err_t motor_driver_disable(MotorDriver *motor);

// Applies a signed duty command between negative and positive max_duty.
esp_err_t motor_driver_set_duty(MotorDriver *motor, float duty);

// Sets PWM to zero while leaving the motor free to coast.
esp_err_t motor_driver_coast(MotorDriver *motor);

// Reports whether motor initialization completed.
bool motor_driver_is_initialized(const MotorDriver *motor);

// Reports whether the motor currently accepts duty commands.
bool motor_driver_is_enabled(const MotorDriver *motor);

// Reports whether the most recent action requested coasting.
bool motor_driver_is_coasting(const MotorDriver *motor);

// Returns the most recently applied signed duty.
float motor_driver_get_current_duty(const MotorDriver *motor);


#ifdef __cplusplus
}
#endif
