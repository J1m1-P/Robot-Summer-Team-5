#include "control/drivetrain.h"
#include "control/drivetrain_kinematics.h"

#include "esp_err.h"
#include "driver/gpio.h"

#include <math.h>
#include <string.h>

#include <robot_common/app_log.h>


// Helper Functions
static float clamp_f(float value, float min_value, float max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static esp_err_t check_motor_id(DrivetrainMotorId id) {
    if (id < 0 || id >= DRIVETRAIN_MOTOR_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static void drivetrain_save_last_duties(Drivetrain *drivetrain, float fl_duty, float fr_duty, float bl_duty, float br_duty) {
    drivetrain->last_duty[DRIVETRAIN_MOTOR_FL] = fl_duty;
    drivetrain->last_duty[DRIVETRAIN_MOTOR_FR] = fr_duty;
    drivetrain->last_duty[DRIVETRAIN_MOTOR_BL] = bl_duty;
    drivetrain->last_duty[DRIVETRAIN_MOTOR_BR] = br_duty;
}

static void drivetrain_save_last_motor_duty(Drivetrain *drivetrain, DrivetrainMotorId motor_id, float duty) {
    drivetrain->last_duty[motor_id] = duty;
}


// Lifecyle
esp_err_t drivetrain_init(Drivetrain *drivetrain, const DrivetrainConfig *config) {
    if (drivetrain == NULL || config == NULL) return ESP_ERR_INVALID_ARG;

    memset(drivetrain, 0, sizeof(Drivetrain));

    drivetrain->config = config;

    // Brake pin initialization
    gpio_config_t brake_gpio_config = {
        .pin_bit_mask = (1ULL << config->brk_pin), 
        .mode = GPIO_MODE_OUTPUT, 
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_DISABLE 
    };

    esp_err_t err = gpio_config(&brake_gpio_config);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to configure brake pin GPIO");
        return err;
    }

    err = gpio_set_level((gpio_num_t)config->brk_pin, 0);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to release brake pin during init");
        return err;
    }

    // Motors and Encoders initialization
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; i++) {
        if (config->motor_configs[i] == NULL) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Missing motor config for motor %d", i);
            return ESP_ERR_INVALID_ARG;
        }

        esp_err_t err = motor_driver_init(&drivetrain->motors[i], config->motor_configs[i]);
        if (err != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to init motor %d", i);
            return err;
        }

        if (config->encoder_configs[i] == NULL) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Missing encoder config for encoder %d", i);
            return ESP_ERR_INVALID_ARG;
        }

        err = encoder_driver_init(&drivetrain->encoders[i], config->encoder_configs[i]);
        if (err != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to init encoder %d", i);
            return err;
        }

        err = encoder_driver_start(&drivetrain->encoders[i]);
        if (err != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to start encoder %d", i);
            return err;
        }
    }

    drivetrain->initialized = true;
    drivetrain->enabled = false;

    drivetrain_save_last_duties(drivetrain, 0.0f, 0.0f, 0.0f, 0.0f);

    APP_LOGI(LOG_TAG_DRIVETRAIN, "Drivetrain initialized");

    return ESP_OK;
}

esp_err_t drivetrain_enable(Drivetrain *drivetrain) {
    if (drivetrain == NULL || !drivetrain->initialized) return ESP_ERR_INVALID_ARG;

    esp_err_t err;

    err = gpio_set_level((gpio_num_t)drivetrain->config->brk_pin, 0);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to release brake");
        return err;
    }

    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; i++) {
        err = motor_driver_enable(&drivetrain->motors[i]);

        if (err != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to enable motor %d", i);
            return err;
        }
    }

    drivetrain->enabled = true;

    APP_LOGI(LOG_TAG_DRIVETRAIN, "Drivetrain enabled");

    return ESP_OK;
}

esp_err_t drivetrain_disable(Drivetrain *drivetrain) {
    if (drivetrain == NULL || !drivetrain->initialized) return ESP_ERR_INVALID_ARG;

    esp_err_t err;

    err = drivetrain_brake(drivetrain);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to brake");
        return err;
    }

    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; i++) {
        err = motor_driver_disable(&drivetrain->motors[i]);
        if (err != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to disable motor %d", i);
            return err;
        }
    }

    drivetrain->enabled = false;

    APP_LOGI(LOG_TAG_DRIVETRAIN, "Drivetrain disabled");

    return ESP_OK;
}


// Motor/Drivetrain Control
esp_err_t drivetrain_brake(Drivetrain *drivetrain) {
    if (drivetrain == NULL || !drivetrain->initialized) return ESP_ERR_INVALID_ARG;

    esp_err_t err;

    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; i++) {
        err = motor_driver_set_duty(&drivetrain->motors[i], 0.0f);
        if (err != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to stop motor %d before brake", i);
            return err;
        }
    }

    drivetrain_save_last_duties(drivetrain, 0.0f, 0.0f, 0.0f, 0.0f);

    err = gpio_set_level((gpio_num_t)drivetrain->config->brk_pin, 1);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to brake");
        return err;
    }

    APP_LOGI(LOG_TAG_DRIVETRAIN, "Drivetrain braked");

    return ESP_OK;
}

esp_err_t drivetrain_coast(Drivetrain *drivetrain) {
    if (drivetrain == NULL || !drivetrain->initialized) return ESP_ERR_INVALID_ARG;

    esp_err_t err;

    err = gpio_set_level((gpio_num_t)drivetrain->config->brk_pin, 0);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to release brake");
        return err;
    }

    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; i++) {
        err = motor_driver_coast(&drivetrain->motors[i]);
        if (err != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to coast motor %d", i);
            return err;
        }
    }

    drivetrain_save_last_duties(drivetrain, 0.0f, 0.0f, 0.0f, 0.0f);

    return ESP_OK;
}

esp_err_t drivetrain_set_motor_duty(Drivetrain *drivetrain, DrivetrainMotorId motor_id, float duty) {
    if (drivetrain == NULL || !drivetrain->initialized || !drivetrain->enabled) return ESP_ERR_INVALID_ARG;
    if (check_motor_id(motor_id) != ESP_OK) return ESP_ERR_INVALID_ARG;
    if (!isfinite(duty)) return ESP_ERR_INVALID_ARG;

    float max_duty = drivetrain->config->max_duty;
    duty = clamp_f(duty, -max_duty, max_duty);

    esp_err_t err;
        
    err = gpio_set_level((gpio_num_t)drivetrain->config->brk_pin, 0);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to release brake");
        return err;
    }

    err = motor_driver_set_duty(&drivetrain->motors[motor_id], duty);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to set duty for motor %d", motor_id);
        return err;
    }

    drivetrain_save_last_motor_duty(drivetrain, motor_id, duty);

    return ESP_OK;
}

esp_err_t drivetrain_set_all_motor_duty(Drivetrain *drivetrain, float fl_duty, float fr_duty, float bl_duty, float br_duty) {
    if (drivetrain == NULL || !drivetrain->initialized || !drivetrain->enabled) return ESP_ERR_INVALID_ARG;
    if (!isfinite(fl_duty) || !isfinite(fr_duty) || !isfinite(bl_duty) || !isfinite(br_duty)) return ESP_ERR_INVALID_ARG;

    float max_duty = drivetrain->config->max_duty;

    float duties[DRIVETRAIN_MOTOR_MAX] = {
        [DRIVETRAIN_MOTOR_FL] = clamp_f(fl_duty, -max_duty, max_duty), 
        [DRIVETRAIN_MOTOR_FR] = clamp_f(fr_duty, -max_duty, max_duty), 
        [DRIVETRAIN_MOTOR_BL] = clamp_f(bl_duty, -max_duty, max_duty), 
        [DRIVETRAIN_MOTOR_BR] = clamp_f(br_duty, -max_duty, max_duty)
    };

    esp_err_t err;

    err = gpio_set_level((gpio_num_t)drivetrain->config->brk_pin, 0);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to release brake");
        return err;
    }

    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; i++) {
        err = motor_driver_set_duty(&drivetrain->motors[i], duties[i]);
        if (err != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to set duty for motor %d", i);
            return err;
        }
    }

    drivetrain_save_last_duties(
        drivetrain,
        duties[DRIVETRAIN_MOTOR_FL],
        duties[DRIVETRAIN_MOTOR_FR],
        duties[DRIVETRAIN_MOTOR_BL],
        duties[DRIVETRAIN_MOTOR_BR]
    );

    return ESP_OK;
}


// Change mapping after read testing
esp_err_t drivetrain_set_body_duty(Drivetrain *drivetrain, float x_duty, float y_duty, float turn_duty) {
    if (drivetrain == NULL || !drivetrain->initialized || !drivetrain->enabled) return ESP_ERR_INVALID_ARG;
    if (!isfinite(x_duty) || !isfinite(y_duty) || !isfinite(turn_duty)) return ESP_ERR_INVALID_ARG;

    esp_err_t err;

    DrivetrainBodyDuty body = {
        .x = x_duty, 
        .y = y_duty, 
        .turn = turn_duty
    };

    DrivetrainWheelDuty wheels = {0};

    DrivetrainKinematicsConfig kine_config = {
        .max_duty = drivetrain->config->max_duty, 
        .wheel_angle_rad = drivetrain->config->wheel_angle_rad
    };

    err = drivetrain_kinematics_body_to_wheels(&kine_config, &body, &wheels);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to convert motor duties");
        return err;
    }

    err = drivetrain_set_all_motor_duty(drivetrain, wheels.fl, wheels.fr, wheels.bl, wheels.br);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to set motor duties");
        return err;
    }
    return ESP_OK;
}

esp_err_t drivetrain_set_forward_duty(Drivetrain *drivetrain, float duty) {
    return drivetrain_set_body_duty(drivetrain, 0.0f, duty, 0.0f);
}

esp_err_t drivetrain_set_turn_duty(Drivetrain *drivetrain, float duty) {
    return drivetrain_set_body_duty(drivetrain, 0.0f, 0.0f, duty);
}

esp_err_t drivetrain_set_strafe_duty(Drivetrain *drivetrain, float duty) {
    return drivetrain_set_body_duty(drivetrain, duty, 0.0f, 0.0f);
}


// Encoder Control 
esp_err_t drivetrain_encoder_update(Drivetrain *drivetrain) {
    if (drivetrain == NULL || !drivetrain->initialized) return ESP_ERR_INVALID_ARG;

    esp_err_t err;
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; i++) {
        err = encoder_driver_update(&drivetrain->encoders[i]);
        if (err != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to update encoder %d", i);
            return err;
        }
    }

    return ESP_OK;
}

int32_t drivetrain_get_encoder_accumulated_count(const Drivetrain *drivetrain, DrivetrainMotorId motor_id) {
    if (drivetrain == NULL || !drivetrain->initialized) return 0;
    if (check_motor_id(motor_id) != ESP_OK) return 0;

    return drivetrain->encoders[motor_id].accumulated_count;
}

float drivetrain_get_encoder_velocity_mps(const Drivetrain *drivetrain, DrivetrainMotorId motor_id) {
    if (drivetrain == NULL || !drivetrain->initialized) return 0.0f;
    if (check_motor_id(motor_id) != ESP_OK) return 0.0f;

    return drivetrain->encoders[motor_id].velocity_mps;
}


// // Watch Dog
// esp_err_t drivetrain_tick(Drivetrain *drivetrain, int64_t now_us) {
//     if (drivetrain->enabled && now_us - drivetrain->)
// }
