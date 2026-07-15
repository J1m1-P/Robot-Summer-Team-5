#include "drivers/motor_driver.h"

#include <stddef.h>
#include <math.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define MOTOR_LEDC_MODE LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_TIMER LEDC_TIMER_0


// Helper Functions
static float clamp_f(float value, float min, float max) {
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

// Precondition: duty must be non-negative and clamped
static uint32_t duty_to_ledc_count(const MotorDriver *motor, float duty) {

    // duty = (1.0f - duty);       // Account for the unknown PWM fliped behaviour

    uint32_t max_count = (1UL << motor->config->pwm_resolution) - 1UL;
    return (uint32_t)(duty * (float)max_count);
}

// Precondition: duty must be non-negative and clamped

// For some unknow reason, the pwm duty logic is inversed, now HIGHER = SLOWER, so we need to invert it 

static void motor_driver_set_pwm(MotorDriver *motor, float duty) {
    uint32_t duty_count = duty_to_ledc_count(motor, duty);

    ledc_set_duty(
        MOTOR_LEDC_MODE, 
        (ledc_channel_t)motor->config->pwm_channel, 
        duty_count
    );

    ledc_update_duty(
        MOTOR_LEDC_MODE, 
        (ledc_channel_t)motor->config->pwm_channel
    );
}

// true = forward, false = backward
static void motor_driver_set_dir(MotorDriver *motor, bool dir) {
    bool pin_level;

    if (motor->config->direction_inverted) {
        pin_level = !dir;
    }
    else {
        pin_level = dir;
    }

    gpio_set_level((gpio_num_t)motor->config->dir_pin, pin_level);
}


// Public API
esp_err_t motor_driver_init(MotorDriver *motor, const MotorDriverConfig *config) {
    if (motor == NULL || config == NULL) return ESP_ERR_INVALID_ARG;
    if (config->max_duty < 0.0f || config->max_duty > 1.0f) return ESP_ERR_INVALID_ARG;
    if (config->pwm_resolution == 0 || config->pwm_resolution > 20) return ESP_ERR_INVALID_ARG;
    if (config->pwm_frequency == 0) return ESP_ERR_INVALID_ARG;
    if (config->pwm_channel >= LEDC_CHANNEL_MAX) return ESP_ERR_INVALID_ARG; 

    motor->config = config;
    motor->initialized = false;
    motor->enabled = false;
    motor->coasting = true;
    motor->current_duty = 0.0f;

    // Configure GPIO pin for direction
    gpio_config_t dir_gpio_config = {
        .pin_bit_mask = (1ULL << config->dir_pin), 
        .mode = GPIO_MODE_OUTPUT, 
        .pull_up_en = GPIO_PULLUP_DISABLE, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE
    };

    esp_err_t err; 

    err = gpio_config(&dir_gpio_config);

    if (err != ESP_OK) {
        return err;
    }

    // Configure PWM timer
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
    motor_driver_set_dir(motor, true);
    motor_driver_set_pwm(motor, 0.0f);

    motor->initialized = true;

    return ESP_OK;
}

esp_err_t motor_driver_enable(MotorDriver *motor) {
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_ARG;

    motor->enabled = true; 

    return ESP_OK;
}

esp_err_t motor_driver_disable(MotorDriver *motor) {
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_ARG;

    motor_driver_coast(motor);
    motor->enabled = false; 

    return ESP_OK;
}

esp_err_t motor_driver_set_duty(MotorDriver *motor, float duty) {
    if (motor == NULL || !motor->initialized || !motor->enabled) return ESP_ERR_INVALID_ARG;
    if (!isfinite(duty)) return ESP_ERR_INVALID_ARG;

    duty = clamp_f(
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
    motor_driver_set_dir(motor, forward);
    motor_driver_set_pwm(motor, pos_duty);

    // Record changes
    motor->current_duty = duty;
    motor->coasting = false;

    return ESP_OK;
}

esp_err_t motor_driver_coast(MotorDriver *motor) {
    if (motor == NULL || !motor->initialized) return ESP_ERR_INVALID_ARG;
    
    motor_driver_set_pwm(motor, 0.0f);

    motor->current_duty = 0.0f;
    motor->coasting = true; 

    return ESP_OK;
}

bool motor_driver_is_initialized(const MotorDriver *motor) {
    if (motor == NULL) return false; 
    return motor->initialized;
}

bool motor_driver_is_enabled(const MotorDriver *motor) {
    if (motor == NULL) return false;
    return motor->enabled;
}

bool motor_driver_is_coasting(const MotorDriver *motor) {
    if (motor == NULL) return false; 
    return motor->coasting;
}

float motor_driver_get_current_duty(const MotorDriver *motor) {
    if (motor == NULL || !motor->initialized) return 0.0f; 
    return motor->current_duty;
}