/* Implements ESP32 GPIO and LEDC control for one bidirectional DC motor. */
#include "drivers/motor/motor_driver.h"

#include <stddef.h>
#include <math.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#include <robot_common/math_utils.h>

// Shared LEDC speed mode and timer used by all drivetrain motor channels.
#define MOTOR_LEDC_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_TIMER LEDC_TIMER_0

static bool s_ledc_timer_initialized = false;

// Validates GPIOs, PWM settings, duty limits, and channel range.
bool motor_driver_config_is_valid(const MotorDriverConfig *config) {
    if (config == NULL) return false;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(config->pwm_pin)) return false;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(config->dir_pin)) return false;
    if (config->pwm_pin == config->dir_pin) return false;
    if (!isfinite(config->max_duty) || config->max_duty <= 0.0f || config->max_duty > 1.0f) {
        return false;
    }
    if (config->pwm_resolution == 0 || config->pwm_resolution > 20) return false;
    if (config->pwm_frequency == 0) return false;
    if (config->pwm_channel >= LEDC_CHANNEL_MAX) return false;
    return true;
}

// Latches a low level before enabling the GPIO output to avoid a startup pulse.
esp_err_t motor_driver_hold_pwm_low(const MotorDriverConfig *config) {
    if (!motor_driver_config_is_valid(config)) return ESP_ERR_INVALID_ARG;

    esp_err_t err = gpio_set_level((gpio_num_t)config->pwm_pin, 0);
    if (err != ESP_OK) return err;

    gpio_config_t pwm_gpio_config = {
        .pin_bit_mask = (1ULL << config->pwm_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    err = gpio_config(&pwm_gpio_config);
    if (err != ESP_OK) return err;

    return gpio_set_level((gpio_num_t)config->pwm_pin, 0);
}

// Converts a non-negative, clamped duty fraction into an LEDC counter value.
static uint32_t duty_to_ledc_count(const MotorDriver *motor, float duty) {
    const uint32_t max_count = (1UL << motor->config->pwm_resolution) - 1UL;
    return (uint32_t)(duty * (float)max_count);
}

// Applies a non-negative, clamped duty to the configured PWM channel.
static esp_err_t motor_driver_set_pwm(MotorDriver *motor, float duty) {
    uint32_t duty_count = duty_to_ledc_count(motor, duty);

    esp_err_t err = ledc_set_duty(
        MOTOR_LEDC_MODE, 
        (ledc_channel_t)motor->config->pwm_channel, 
        duty_count
    );
    if (err != ESP_OK) return err;

    return ledc_update_duty(
        MOTOR_LEDC_MODE, 
        (ledc_channel_t)motor->config->pwm_channel
    );
}

// Applies a logical direction while honoring the motor's inversion setting.
static esp_err_t motor_driver_set_dir(MotorDriver *motor, bool dir) {
    const bool pin_level = motor->config->direction_inverted ? !dir : dir;
    return gpio_set_level((gpio_num_t)motor->config->dir_pin, pin_level);
}


// Configures direction GPIO and the shared LEDC timer and channel.
esp_err_t motor_driver_init(MotorDriver *motor, const MotorDriverConfig *config) {
    if (motor == NULL || !motor_driver_config_is_valid(config)) return ESP_ERR_INVALID_ARG;

    motor->config = config;
    motor->initialized = false;
    motor->enabled = false;
    motor->coasting = true;
    motor->current_duty = 0.0f;

    // Take control of PWM first. This also makes direct motor initialization
    // safe when the caller did not perform the optional early startup hold.
    esp_err_t err = motor_driver_hold_pwm_low(config);
    if (err != ESP_OK) return err;

    // Configure GPIO pin for direction
    gpio_config_t dir_gpio_config = {
        .pin_bit_mask = (1ULL << config->dir_pin), 
        .mode = GPIO_MODE_OUTPUT, 
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE
    };

    err = gpio_config(&dir_gpio_config);

    if (err != ESP_OK) {
        return err;
    }

    // Configure the shared PWM timer once for all motor channels.
    if (!s_ledc_timer_initialized) {
        ledc_timer_config_t pwm_timer_config = {
            .speed_mode = MOTOR_LEDC_MODE, 
            .timer_num = MOTOR_LEDC_TIMER, 
            .duty_resolution = (ledc_timer_bit_t)config->pwm_resolution, 
            .freq_hz = config->pwm_frequency, 
            .clk_cfg = LEDC_AUTO_CLK
        };

        err = ledc_timer_config(&pwm_timer_config);
        if (err != ESP_OK) {
            return err;
        }

        s_ledc_timer_initialized = true;
    }

    // Configure PWM channel
    ledc_channel_config_t pwm_channel_config = {
        .gpio_num = config->pwm_pin, 
        .speed_mode = MOTOR_LEDC_MODE, 
        .channel = (ledc_channel_t)config->pwm_channel, 
        .intr_type = LEDC_INTR_DISABLE, 
        .timer_sel = MOTOR_LEDC_TIMER, 
        .duty = 0, 
        .hpoint = 0
    };

    err = ledc_channel_config(&pwm_channel_config);
    if (err != ESP_OK) {
        return err;
    }

    //Set default states
    err = motor_driver_set_dir(motor, true);
    if (err != ESP_OK) return err;

    err = motor_driver_set_pwm(motor, 0.0f);
    if (err != ESP_OK) return err;

    motor->initialized = true;

    return ESP_OK;
}

// Enables duty commands for an initialized motor.
esp_err_t motor_driver_enable(MotorDriver *motor) {
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_ARG;

    motor->enabled = true; 

    return ESP_OK;
}

// Zeros PWM and disables future duty commands until re-enabled.
esp_err_t motor_driver_disable(MotorDriver *motor) {
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_ARG;

    esp_err_t err = motor_driver_coast(motor);
    if (err != ESP_OK) return err;

    motor->enabled = false; 

    return ESP_OK;
}

// Clamps and applies signed motor duty through direction and PWM outputs.
esp_err_t motor_driver_set_duty(MotorDriver *motor, float duty) {
    if (motor == NULL || !motor->initialized || !motor->enabled) return ESP_ERR_INVALID_ARG;
    if (!isfinite(duty)) return ESP_ERR_INVALID_ARG;

    duty = clamp(
        duty, 
        -motor->config->max_duty, 
        motor->config->max_duty
    );

    if (duty == 0.0f) {
        return motor_driver_coast(motor);
    }

    bool forward = duty > 0.0f;
    float pos_duty = fabsf(duty);

    // Make changes
    esp_err_t err = motor_driver_set_dir(motor, forward);
    if (err != ESP_OK) return err;

    err = motor_driver_set_pwm(motor, pos_duty);
    if (err != ESP_OK) return err;

    // Record changes
    motor->current_duty = duty;
    motor->coasting = false;

    return ESP_OK;
}

// Zeros PWM and records that the motor is in a coasting state.
esp_err_t motor_driver_coast(MotorDriver *motor) {
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_ARG;
    
    esp_err_t err = motor_driver_set_pwm(motor, 0.0f);
    if (err != ESP_OK) return err;

    motor->current_duty = 0.0f;
    motor->coasting = true; 

    return ESP_OK;
}

// Reports whether motor initialization completed.
bool motor_driver_is_initialized(const MotorDriver *motor) {
    if (motor == NULL) return false; 
    return motor->initialized;
}

// Reports whether the motor currently accepts duty commands.
bool motor_driver_is_enabled(const MotorDriver *motor) {
    if (motor == NULL) return false;
    return motor->enabled;
}

// Reports whether the last motor action requested coasting.
bool motor_driver_is_coasting(const MotorDriver *motor) {
    if (motor == NULL) return false; 
    return motor->coasting;
}

// Returns the most recently applied signed duty or zero for a null motor.
float motor_driver_get_current_duty(const MotorDriver *motor) {
    if (motor == NULL || !motor->initialized) return 0.0f; 
    return motor->current_duty;
}
