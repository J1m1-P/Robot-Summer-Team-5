#include "control/drivetrain.h"
#include "control/drivetrain_kinematics.h"

#include <math.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_timer.h"

#include <robot_common/app_log.h>
#include <robot_common/math_utils.h>

static esp_err_t check_motor_id(DrivetrainMotorId id) {
    if (id < 0 || id >= DRIVETRAIN_MOTOR_MAX) return ESP_ERR_INVALID_ARG;
    return ESP_OK;
}

static void record_first_error(esp_err_t *first_error, esp_err_t error) {
    if (*first_error == ESP_OK && error != ESP_OK) {
        *first_error = error;
    }
}

static void drivetrain_save_last_duties(Drivetrain *drivetrain, float fl_duty,
                                         float fr_duty, float bl_duty, float br_duty) {
    drivetrain->last_duty[DRIVETRAIN_MOTOR_FL] = fl_duty;
    drivetrain->last_duty[DRIVETRAIN_MOTOR_FR] = fr_duty;
    drivetrain->last_duty[DRIVETRAIN_MOTOR_BL] = bl_duty;
    drivetrain->last_duty[DRIVETRAIN_MOTOR_BR] = br_duty;
}

static void drivetrain_record_command(Drivetrain *drivetrain) {
    drivetrain->last_command_us = esp_timer_get_time();
    drivetrain->command_timeout_active = false;
}

static bool drivetrain_config_is_valid(const DrivetrainConfig *config) {
    if (config == NULL) return false;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(config->brk_pin)) return false;
    if (!isfinite(config->max_duty) || config->max_duty <= 0.0f ||
        config->max_duty > 1.0f) {
        return false;
    }
    if (!isfinite(config->wheel_angle_rad)) return false;

    float sin_angle = sinf(config->wheel_angle_rad);
    float cos_angle = cosf(config->wheel_angle_rad);
    if (fabsf(sin_angle) < 1e-6f || fabsf(cos_angle) < 1e-6f) return false;

    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        const MotorDriverConfig *motor_config = config->motor_configs[index];
        const EncoderDriverConfig *encoder_config = config->encoder_configs[index];

        if (!motor_driver_config_is_valid(motor_config)) return false;
        if (!encoder_driver_config_is_valid(encoder_config)) return false;
        if (config->max_duty > motor_config->max_duty) return false;
        if (encoder_config->id != (EncoderId)index) return false;
        if (motor_config->pwm_pin == config->brk_pin ||
            motor_config->dir_pin == config->brk_pin) {
            return false;
        }

        if (index > 0) {
            const MotorDriverConfig *first_motor = config->motor_configs[0];
            if (motor_config->pwm_frequency != first_motor->pwm_frequency ||
                motor_config->pwm_resolution != first_motor->pwm_resolution) {
                return false;
            }
        }

        for (int previous = 0; previous < index; previous++) {
            if (motor_config->pwm_channel ==
                config->motor_configs[previous]->pwm_channel) {
                return false;
            }
            if (encoder_config->pcnt_unit ==
                config->encoder_configs[previous]->pcnt_unit) {
                return false;
            }
        }
    }

    return true;
}

static void drivetrain_cleanup_failed_init(Drivetrain *drivetrain) {
    for (int index = DRIVETRAIN_MOTOR_MAX - 1; index >= 0; index--) {
        EncoderDriver *encoder = &drivetrain->encoders[index];
        if (encoder_driver_is_enabled(encoder)) {
            esp_err_t error = encoder_driver_stop(encoder);
            if (error != ESP_OK) {
                APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to stop encoder %d during init cleanup: %s",
                         index, esp_err_to_name(error));
            }
        }
    }

    for (int index = DRIVETRAIN_MOTOR_MAX - 1; index >= 0; index--) {
        MotorDriver *motor = &drivetrain->motors[index];
        if (motor_driver_is_initialized(motor)) {
            esp_err_t error = motor_driver_disable(motor);
            if (error != ESP_OK) {
                APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to disable motor %d during init cleanup: %s",
                         index, esp_err_to_name(error));
            }
        }
    }

    esp_err_t brake_error =
        gpio_set_level((gpio_num_t)drivetrain->config->brk_pin, 1);
    if (brake_error != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to engage brake during init cleanup: %s",
                 esp_err_to_name(brake_error));
    }

    memset(drivetrain, 0, sizeof(*drivetrain));
}

static void drivetrain_cleanup_failed_enable(Drivetrain *drivetrain,
                                              int enabled_motor_count) {
    for (int index = enabled_motor_count - 1; index >= 0; index--) {
        esp_err_t error = motor_driver_disable(&drivetrain->motors[index]);
        if (error != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to disable motor %d during enable cleanup: %s",
                     index, esp_err_to_name(error));
        }
    }

    esp_err_t brake_error =
        gpio_set_level((gpio_num_t)drivetrain->config->brk_pin, 1);
    if (brake_error != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to engage brake during enable cleanup: %s",
                 esp_err_to_name(brake_error));
    }

    drivetrain->enabled = false;
    drivetrain->brake_engaged = true;
    drivetrain->last_command_us = 0;
    drivetrain->command_timeout_active = false;
    drivetrain_save_last_duties(drivetrain, 0.0f, 0.0f, 0.0f, 0.0f);
}

esp_err_t drivetrain_init(Drivetrain *drivetrain, const DrivetrainConfig *config) {
    if (drivetrain == NULL || !drivetrain_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (drivetrain->initialized) return ESP_ERR_INVALID_STATE;

    memset(drivetrain, 0, sizeof(*drivetrain));
    drivetrain->config = config;

    gpio_config_t brake_gpio_config = {
        .pin_bit_mask = (1ULL << config->brk_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t error = gpio_config(&brake_gpio_config);
    if (error != ESP_OK) {
        memset(drivetrain, 0, sizeof(*drivetrain));
        return error;
    }

    error = gpio_set_level((gpio_num_t)config->brk_pin, 1);
    if (error != ESP_OK) {
        memset(drivetrain, 0, sizeof(*drivetrain));
        return error;
    }
    drivetrain->brake_engaged = true;

    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        error = motor_driver_init(&drivetrain->motors[index],
                                  config->motor_configs[index]);
        if (error != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to initialize motor %d: %s", index,
                     esp_err_to_name(error));
            drivetrain_cleanup_failed_init(drivetrain);
            return error;
        }

        error = encoder_driver_init(&drivetrain->encoders[index],
                                    config->encoder_configs[index]);
        if (error != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to initialize encoder %d: %s", index,
                     esp_err_to_name(error));
            drivetrain_cleanup_failed_init(drivetrain);
            return error;
        }

        error = encoder_driver_start(&drivetrain->encoders[index]);
        if (error != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to start encoder %d: %s", index,
                     esp_err_to_name(error));
            drivetrain_cleanup_failed_init(drivetrain);
            return error;
        }
    }

    drivetrain->initialized = true;
    drivetrain->enabled = false;
    drivetrain->brake_engaged = true;
    drivetrain->last_command_us = 0;
    drivetrain->command_timeout_active = false;
    drivetrain_save_last_duties(drivetrain, 0.0f, 0.0f, 0.0f, 0.0f);

    APP_LOGI(LOG_TAG_DRIVETRAIN, "Drivetrain initialized with brake engaged");
    return ESP_OK;
}

esp_err_t drivetrain_enable(Drivetrain *drivetrain) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->initialized) return ESP_ERR_INVALID_STATE;
    if (drivetrain->enabled) return ESP_ERR_INVALID_STATE;

    int enabled_motor_count = 0;
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        esp_err_t error = motor_driver_enable(&drivetrain->motors[index]);
        if (error != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to enable motor %d: %s", index,
                     esp_err_to_name(error));
            drivetrain_cleanup_failed_enable(drivetrain, enabled_motor_count);
            return error;
        }
        enabled_motor_count++;
    }

    esp_err_t error =
        gpio_set_level((gpio_num_t)drivetrain->config->brk_pin, 0);
    if (error != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to release brake: %s",
                 esp_err_to_name(error));
        drivetrain_cleanup_failed_enable(drivetrain, enabled_motor_count);
        return error;
    }

    drivetrain->enabled = true;
    drivetrain->brake_engaged = false;
    drivetrain->last_command_us = esp_timer_get_time();
    drivetrain->command_timeout_active = false;

    APP_LOGI(LOG_TAG_DRIVETRAIN, "Drivetrain enabled and brake released");
    return ESP_OK;
}

esp_err_t drivetrain_disable(Drivetrain *drivetrain) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->initialized) return ESP_ERR_INVALID_STATE;
    if (!drivetrain->enabled && drivetrain->brake_engaged) return ESP_OK;
    return drivetrain_brake(drivetrain);
}

esp_err_t drivetrain_brake(Drivetrain *drivetrain) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t first_error = ESP_OK;

    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        esp_err_t error = motor_driver_coast(&drivetrain->motors[index]);
        record_first_error(&first_error, error);
    }

    esp_err_t brake_error =
        gpio_set_level((gpio_num_t)drivetrain->config->brk_pin, 1);
    record_first_error(&first_error, brake_error);

    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        if (motor_driver_is_enabled(&drivetrain->motors[index])) {
            esp_err_t error = motor_driver_disable(&drivetrain->motors[index]);
            record_first_error(&first_error, error);
        }
    }

    drivetrain->enabled = false;
    drivetrain->brake_engaged = brake_error == ESP_OK;
    drivetrain->last_command_us = 0;
    drivetrain->command_timeout_active = false;
    drivetrain_save_last_duties(drivetrain, 0.0f, 0.0f, 0.0f, 0.0f);

    if (first_error != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to fully brake drivetrain: %s",
                 esp_err_to_name(first_error));
        return first_error;
    }

    APP_LOGI(LOG_TAG_DRIVETRAIN, "Drivetrain braked and disabled");
    return ESP_OK;
}

esp_err_t drivetrain_coast(Drivetrain *drivetrain) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t first_error = ESP_OK;
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        esp_err_t error = motor_driver_coast(&drivetrain->motors[index]);
        record_first_error(&first_error, error);
    }

    if (first_error != ESP_OK) {
        drivetrain_brake(drivetrain);
        return first_error;
    }

    esp_err_t error =
        gpio_set_level((gpio_num_t)drivetrain->config->brk_pin, 0);
    if (error != ESP_OK) {
        drivetrain_brake(drivetrain);
        return error;
    }

    drivetrain->brake_engaged = false;
    drivetrain_save_last_duties(drivetrain, 0.0f, 0.0f, 0.0f, 0.0f);
    return ESP_OK;
}

esp_err_t drivetrain_set_motor_duty(Drivetrain *drivetrain,
                                     DrivetrainMotorId motor_id, float duty) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->initialized || !drivetrain->enabled ||
        drivetrain->brake_engaged) {
        return ESP_ERR_INVALID_STATE;
    }
    if (check_motor_id(motor_id) != ESP_OK || !isfinite(duty)) {
        return ESP_ERR_INVALID_ARG;
    }

    duty = clamp(duty, -drivetrain->config->max_duty,
                 drivetrain->config->max_duty);

    esp_err_t error =
        motor_driver_set_duty(&drivetrain->motors[motor_id], duty);
    if (error != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to set duty for motor %d: %s", motor_id,
                 esp_err_to_name(error));
        drivetrain_brake(drivetrain);
        return error;
    }

    drivetrain->last_duty[motor_id] = duty;
    drivetrain_record_command(drivetrain);
    return ESP_OK;
}

esp_err_t drivetrain_set_all_motor_duty(Drivetrain *drivetrain, float fl_duty,
                                         float fr_duty, float bl_duty,
                                         float br_duty) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->initialized || !drivetrain->enabled ||
        drivetrain->brake_engaged) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!isfinite(fl_duty) || !isfinite(fr_duty) || !isfinite(bl_duty) ||
        !isfinite(br_duty)) {
        return ESP_ERR_INVALID_ARG;
    }

    float max_duty = drivetrain->config->max_duty;
    float duties[DRIVETRAIN_MOTOR_MAX] = {
        [DRIVETRAIN_MOTOR_FL] = clamp(fl_duty, -max_duty, max_duty),
        [DRIVETRAIN_MOTOR_FR] = clamp(fr_duty, -max_duty, max_duty),
        [DRIVETRAIN_MOTOR_BL] = clamp(bl_duty, -max_duty, max_duty),
        [DRIVETRAIN_MOTOR_BR] = clamp(br_duty, -max_duty, max_duty),
    };

    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        esp_err_t error =
            motor_driver_set_duty(&drivetrain->motors[index], duties[index]);
        if (error != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to set duty for motor %d: %s", index,
                     esp_err_to_name(error));
            drivetrain_brake(drivetrain);
            return error;
        }
    }

    drivetrain_save_last_duties(
        drivetrain, duties[DRIVETRAIN_MOTOR_FL], duties[DRIVETRAIN_MOTOR_FR],
        duties[DRIVETRAIN_MOTOR_BL], duties[DRIVETRAIN_MOTOR_BR]);
    drivetrain_record_command(drivetrain);
    return ESP_OK;
}

esp_err_t drivetrain_set_body_duty(Drivetrain *drivetrain, float x_duty,
                                    float y_duty, float turn_duty) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->initialized || !drivetrain->enabled ||
        drivetrain->brake_engaged) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!isfinite(x_duty) || !isfinite(y_duty) || !isfinite(turn_duty)) {
        return ESP_ERR_INVALID_ARG;
    }

    DrivetrainBodyDuty body = {
        .x = x_duty,
        .y = y_duty,
        .turn = turn_duty,
    };
    DrivetrainWheelDuty wheels = {0};
    DrivetrainKinematicsConfig kinematics_config = {
        .wheel_angle_rad = drivetrain->config->wheel_angle_rad,
        .max_duty = drivetrain->config->max_duty,
    };

    esp_err_t error = drivetrain_kinematics_body_to_wheels(
        &kinematics_config, &body, &wheels);
    if (error != ESP_OK) return error;

    return drivetrain_set_all_motor_duty(
        drivetrain, wheels.fl, wheels.fr, wheels.bl, wheels.br);
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

esp_err_t drivetrain_encoder_update(Drivetrain *drivetrain) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->initialized) return ESP_ERR_INVALID_STATE;

    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        esp_err_t error = encoder_driver_update(&drivetrain->encoders[index]);
        if (error != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Failed to update encoder %d: %s", index,
                     esp_err_to_name(error));
            return error;
        }
    }

    return ESP_OK;
}

int32_t drivetrain_get_encoder_accumulated_count(
    const Drivetrain *drivetrain, DrivetrainMotorId motor_id) {
    if (drivetrain == NULL || !drivetrain->initialized) return 0;
    if (check_motor_id(motor_id) != ESP_OK) return 0;
    return drivetrain->encoders[motor_id].accumulated_count;
}

float drivetrain_get_encoder_velocity_mps(const Drivetrain *drivetrain,
                                           DrivetrainMotorId motor_id) {
    if (drivetrain == NULL || !drivetrain->initialized) return 0.0f;
    if (check_motor_id(motor_id) != ESP_OK) return 0.0f;
    return drivetrain->encoders[motor_id].velocity_mps;
}

esp_err_t drivetrain_tick(Drivetrain *drivetrain, int64_t now_us) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->initialized) return ESP_ERR_INVALID_STATE;
    if (now_us < 0) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->enabled) return ESP_OK;
    if (drivetrain->command_timeout_active) return ESP_OK;

    if (drivetrain->last_command_us == 0 ||
        now_us - drivetrain->last_command_us > DRIVETRAIN_COMMAND_TIMEOUT_US) {
        esp_err_t error = drivetrain_coast(drivetrain);
        if (error != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Command timeout safe-stop failed: %s",
                     esp_err_to_name(error));
            return error;
        }

        drivetrain->command_timeout_active = true;
        APP_LOGW(LOG_TAG_DRIVETRAIN, "Motor command timed out; drivetrain coasting");
    }

    return ESP_OK;
}
