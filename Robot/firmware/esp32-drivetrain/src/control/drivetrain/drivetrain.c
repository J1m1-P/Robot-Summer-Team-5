/* Implements velocity control, hardware lifecycle, telemetry, and safe stopping. */
#include "control/drivetrain/drivetrain.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_timer.h"

#include <robot_common/app_log.h>

// Checks wheel identifiers before using them as array indexes.
static bool motor_id_is_valid(DrivetrainMotorId id) {
    return id >= DRIVETRAIN_MOTOR_FL && id < DRIVETRAIN_MOTOR_MAX;
}

// Preserves the first failure while allowing every cleanup action to run.
static void record_first_error(esp_err_t *first_error, esp_err_t error) {
    if (*first_error == ESP_OK && error != ESP_OK) *first_error = error;
}

// Clamps a signed duty to the configured drivetrain safety ceiling.
static float clamp_duty(float duty, float max_duty) {
    if (duty > max_duty) return max_duty;
    if (duty < -max_duty) return -max_duty;
    return duty;
}

// Clears velocity targets, PI history, and wheel-output telemetry.
static void reset_control_state(Drivetrain *drivetrain) {
    memset(&drivetrain->status.target_body, 0, sizeof(drivetrain->status.target_body));
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        wheel_velocity_pi_reset(&drivetrain->devices.wheel_pi[index]);
        drivetrain->control.target_wheel_mps[index] = 0.0f;
        drivetrain->control.last_duty[index] = 0.0f;
    }
}

// Checks PI output bounds against the final drivetrain duty limit.
static bool pi_config_fits_drivetrain(
    const WheelVelocityPiConfig *pi_config,
    float max_duty
) {
    return wheel_velocity_pi_config_is_valid(pi_config) &&
           pi_config->output_min >= -max_duty &&
           pi_config->output_max <= max_duty;
}

// Validates one positive finite motion or timing bound.
static bool positive_finite(float value) {
    return isfinite(value) && value > 0.0f;
}

// Validates hardware assignments, geometry, controller, and safety bounds.
static bool drivetrain_config_is_valid(const DrivetrainConfig *config) {
    if (config == NULL || !GPIO_IS_VALID_OUTPUT_GPIO(config->brake_pin)) return false;
    if (!positive_finite(config->max_duty) || config->max_duty > 1.0f ||
        !positive_finite(config->max_vx_mps) ||
        !positive_finite(config->max_vy_mps) ||
        !positive_finite(config->max_omega_rad_s) ||
        !positive_finite(config->max_control_dt_s) ||
        config->command_timeout_us <= 0) {
        return false;
    }
    if (!pi_config_fits_drivetrain(&config->wheel_pi, config->max_duty)) return false;

    DrivetrainBodyVelocity zero_body = {0};
    DrivetrainWheelVelocity validation_output = {0};
    if (drivetrain_kinematics_body_to_wheel_velocities(
            &config->kinematics, &zero_body, &validation_output) != ESP_OK) {
        return false;
    }

    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        const MotorDriverConfig *motor_config = config->motor_configs[index];
        const EncoderDriverConfig *encoder_config = config->encoder_configs[index];
        if (!motor_driver_config_is_valid(motor_config) ||
            !encoder_driver_config_is_valid(encoder_config)) {
            return false;
        }
        if (config->max_duty > motor_config->max_duty) return false;
        if (encoder_config->id != (EncoderId)index) return false;
        if (motor_config->pwm_pin == config->brake_pin ||
            motor_config->dir_pin == config->brake_pin) {
            return false;
        }

        if (index > 0) {
            const MotorDriverConfig *first = config->motor_configs[0];
            if (motor_config->pwm_frequency != first->pwm_frequency ||
                motor_config->pwm_resolution != first->pwm_resolution) {
                return false;
            }
        }
        for (int previous = 0; previous < index; ++previous) {
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

// Restores a braked, empty object after partial initialization fails.
static void cleanup_failed_init(Drivetrain *drivetrain) {
    for (int index = DRIVETRAIN_MOTOR_MAX - 1; index >= 0; --index) {
        EncoderDriver *encoder = &drivetrain->devices.encoders[index];
        MotorDriver *motor = &drivetrain->devices.motors[index];
        if (encoder_driver_is_enabled(encoder)) encoder_driver_stop(encoder);
        if (motor_driver_is_initialized(motor)) motor_driver_disable(motor);
    }
    gpio_set_level((gpio_num_t)drivetrain->config->brake_pin, 1);
    memset(drivetrain, 0, sizeof(*drivetrain));
}

// Disables motors already enabled when a later enable step fails.
static void cleanup_failed_enable(Drivetrain *drivetrain, int enabled_count) {
    for (int index = enabled_count - 1; index >= 0; --index) {
        motor_driver_disable(&drivetrain->devices.motors[index]);
    }
    gpio_set_level((gpio_num_t)drivetrain->config->brake_pin, 1);
    drivetrain->status.enabled = false;
    drivetrain->status.brake_engaged = true;
    drivetrain->status.command_timeout_active = false;
    drivetrain->control.last_command_us = 0;
    drivetrain->control.last_update_us = 0;
    reset_control_state(drivetrain);
}

// Updates all encoder estimates or brakes on the first failure.
static esp_err_t update_encoders(Drivetrain *drivetrain) {
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        esp_err_t error = encoder_driver_update(&drivetrain->devices.encoders[index]);
        if (error != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Encoder %d update failed: %s", index,
                     esp_err_to_name(error));
            drivetrain_brake(drivetrain);
            return error;
        }
    }
    return ESP_OK;
}

// Applies one private logical-order wheel-duty command or brakes on failure.
static esp_err_t apply_wheel_duties(Drivetrain *drivetrain, const float duties[]) {
    float bounded[DRIVETRAIN_MOTOR_MAX] = {0};
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        if (!isfinite(duties[index])) return ESP_ERR_INVALID_ARG;
        bounded[index] = clamp_duty(duties[index], drivetrain->config->max_duty);
    }

    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        esp_err_t error = motor_driver_set_duty(
            &drivetrain->devices.motors[index], bounded[index]);
        if (error != ESP_OK) {
            APP_LOGE(LOG_TAG_DRIVETRAIN, "Motor %d duty failed: %s", index,
                     esp_err_to_name(error));
            drivetrain_brake(drivetrain);
            return error;
        }
    }
    memcpy(drivetrain->control.last_duty, bounded, sizeof(bounded));
    return ESP_OK;
}

// Initializes the brake plus all motor and encoder devices with rollback.
esp_err_t drivetrain_init(Drivetrain *drivetrain, const DrivetrainConfig *config) {
    if (drivetrain == NULL || !drivetrain_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (drivetrain->status.initialized) return ESP_ERR_INVALID_STATE;

    memset(drivetrain, 0, sizeof(*drivetrain));
    drivetrain->config = config;
    drivetrain->control.active_pi_config = config->wheel_pi;

    gpio_config_t brake_config = {0};
    brake_config.pin_bit_mask = 1ULL << config->brake_pin;
    brake_config.mode = GPIO_MODE_OUTPUT;
    brake_config.pull_up_en = GPIO_PULLUP_DISABLE;
    brake_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    brake_config.intr_type = GPIO_INTR_DISABLE;

    esp_err_t error = gpio_config(&brake_config);
    if (error != ESP_OK) {
        memset(drivetrain, 0, sizeof(*drivetrain));
        return error;
    }
    error = gpio_set_level((gpio_num_t)config->brake_pin, 1);
    if (error != ESP_OK) {
        memset(drivetrain, 0, sizeof(*drivetrain));
        return error;
    }
    drivetrain->status.brake_engaged = true;

    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        error = motor_driver_init(&drivetrain->devices.motors[index],
                                  config->motor_configs[index]);
        if (error != ESP_OK) {
            cleanup_failed_init(drivetrain);
            return error;
        }
        error = encoder_driver_init(&drivetrain->devices.encoders[index],
                                    config->encoder_configs[index]);
        if (error != ESP_OK) {
            cleanup_failed_init(drivetrain);
            return error;
        }
        error = encoder_driver_start(&drivetrain->devices.encoders[index]);
        if (error != ESP_OK) {
            cleanup_failed_init(drivetrain);
            return error;
        }
    }

    drivetrain->status.initialized = true;
    APP_LOGI(LOG_TAG_DRIVETRAIN, "Drivetrain initialized with brake engaged");
    return ESP_OK;
}

// Enables every motor, resets controller history, and releases the brake.
esp_err_t drivetrain_enable(Drivetrain *drivetrain) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->status.initialized || drivetrain->status.enabled) {
        return ESP_ERR_INVALID_STATE;
    }

    int enabled_count = 0;
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        esp_err_t error = motor_driver_enable(&drivetrain->devices.motors[index]);
        if (error != ESP_OK) {
            cleanup_failed_enable(drivetrain, enabled_count);
            return error;
        }
        ++enabled_count;
    }

    esp_err_t error = gpio_set_level((gpio_num_t)drivetrain->config->brake_pin, 0);
    if (error != ESP_OK) {
        cleanup_failed_enable(drivetrain, enabled_count);
        return error;
    }

    reset_control_state(drivetrain);
    drivetrain->status.enabled = true;
    drivetrain->status.brake_engaged = false;
    drivetrain->status.command_timeout_active = false;
    drivetrain->control.last_command_us = esp_timer_get_time();
    drivetrain->control.last_update_us = drivetrain->control.last_command_us;
    APP_LOGI(LOG_TAG_DRIVETRAIN, "Drivetrain enabled");
    return ESP_OK;
}

// Delegates disable behavior to the hardware-brake safe state.
esp_err_t drivetrain_disable(Drivetrain *drivetrain) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->status.initialized) return ESP_ERR_INVALID_STATE;
    if (!drivetrain->status.enabled && drivetrain->status.brake_engaged) return ESP_OK;
    return drivetrain_brake(drivetrain);
}

// Stores a bounded body target and refreshes the command watchdog.
esp_err_t drivetrain_set_body_velocity(
    Drivetrain *drivetrain,
    float vx_mps,
    float vy_mps,
    float omega_rad_s
) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->status.initialized || !drivetrain->status.enabled ||
        drivetrain->status.brake_engaged) {
        return ESP_ERR_INVALID_STATE;
    }
    if (!isfinite(vx_mps) || !isfinite(vy_mps) || !isfinite(omega_rad_s) ||
        fabsf(vx_mps) > drivetrain->config->max_vx_mps ||
        fabsf(vy_mps) > drivetrain->config->max_vy_mps ||
        fabsf(omega_rad_s) > drivetrain->config->max_omega_rad_s) {
        return ESP_ERR_INVALID_ARG;
    }

    drivetrain->status.target_body.vx = vx_mps;
    drivetrain->status.target_body.vy = vy_mps;
    drivetrain->status.target_body.omega = omega_rad_s;
    drivetrain->control.last_command_us = esp_timer_get_time();
    drivetrain->status.command_timeout_active = false;
    return ESP_OK;
}

// Requests a controlled stop through the closed-loop velocity path.
esp_err_t drivetrain_stop(Drivetrain *drivetrain) {
    return drivetrain_set_body_velocity(drivetrain, 0.0f, 0.0f, 0.0f);
}

// Runs one complete bounded-time closed-loop velocity-control iteration.
esp_err_t drivetrain_update(Drivetrain *drivetrain, int64_t now_us) {
    if (drivetrain == NULL || now_us < 0) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->status.initialized) return ESP_ERR_INVALID_STATE;
    if (!drivetrain->status.enabled) return ESP_OK;
    if (now_us < drivetrain->control.last_command_us) return ESP_ERR_INVALID_ARG;

    if (drivetrain->control.last_command_us == 0 ||
        now_us - drivetrain->control.last_command_us >
            drivetrain->config->command_timeout_us) {
        if (!drivetrain->status.command_timeout_active) {
            esp_err_t error = drivetrain_coast(drivetrain);
            if (error != ESP_OK) return error;
            drivetrain->status.command_timeout_active = true;
            APP_LOGW(LOG_TAG_DRIVETRAIN, "Velocity command timed out; drivetrain coasting");
        }
        return ESP_OK;
    }

    if (drivetrain->control.last_update_us <= 0) {
        drivetrain->control.last_update_us = now_us;
        return ESP_OK;
    }
    if (now_us < drivetrain->control.last_update_us) return ESP_ERR_INVALID_ARG;
    if (now_us == drivetrain->control.last_update_us) return ESP_OK;

    const float dt_s = (float)(now_us - drivetrain->control.last_update_us) / 1000000.0f;
    if (!isfinite(dt_s) || dt_s > drivetrain->config->max_control_dt_s) {
        drivetrain_coast(drivetrain);
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t error = update_encoders(drivetrain);
    if (error != ESP_OK) return error;

    DrivetrainWheelVelocity wheel_rad_s = {0};
    error = drivetrain_kinematics_body_to_wheel_velocities(
        &drivetrain->config->kinematics,
        &drivetrain->status.target_body,
        &wheel_rad_s);
    if (error != ESP_OK) {
        drivetrain_brake(drivetrain);
        return error;
    }

    const float radius = drivetrain->config->kinematics.wheel_radius_m;
    drivetrain->control.target_wheel_mps[DRIVETRAIN_MOTOR_FL] = wheel_rad_s.fl * radius;
    drivetrain->control.target_wheel_mps[DRIVETRAIN_MOTOR_FR] = wheel_rad_s.fr * radius;
    drivetrain->control.target_wheel_mps[DRIVETRAIN_MOTOR_BL] = wheel_rad_s.bl * radius;
    drivetrain->control.target_wheel_mps[DRIVETRAIN_MOTOR_BR] = wheel_rad_s.br * radius;

    float duties[DRIVETRAIN_MOTOR_MAX] = {0};
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        const float measured = encoder_driver_get_velocity_mps(
            &drivetrain->devices.encoders[index]);
        error = wheel_velocity_pi_update(
            &drivetrain->devices.wheel_pi[index],
            &drivetrain->control.active_pi_config,
            drivetrain->control.target_wheel_mps[index],
            measured,
            dt_s,
            &duties[index]);
        if (error != ESP_OK) {
            drivetrain_brake(drivetrain);
            return error;
        }
    }

    error = apply_wheel_duties(drivetrain, duties);
    if (error == ESP_OK) drivetrain->control.last_update_us = now_us;
    return error;
}

// Validates and installs live tuning gains, then clears prior PI history.
esp_err_t drivetrain_set_wheel_pi_config(
    Drivetrain *drivetrain,
    const WheelVelocityPiConfig *config
) {
    if (drivetrain == NULL || config == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->status.initialized) return ESP_ERR_INVALID_STATE;
    if (!pi_config_fits_drivetrain(config, drivetrain->config->max_duty)) {
        return ESP_ERR_INVALID_ARG;
    }
    drivetrain->control.active_pi_config = *config;
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        wheel_velocity_pi_reset(&drivetrain->devices.wheel_pi[index]);
    }
    return ESP_OK;
}

// Copies the live gains without exposing mutable controller ownership.
esp_err_t drivetrain_get_wheel_pi_config(
    const Drivetrain *drivetrain,
    WheelVelocityPiConfig *config_out
) {
    if (drivetrain == NULL || config_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->status.initialized) return ESP_ERR_INVALID_STATE;
    *config_out = drivetrain->control.active_pi_config;
    return ESP_OK;
}

// Copies the caller-facing status snapshot.
esp_err_t drivetrain_get_status(
    const Drivetrain *drivetrain,
    DrivetrainStatus *status_out
) {
    if (drivetrain == NULL || status_out == NULL) return ESP_ERR_INVALID_ARG;
    *status_out = drivetrain->status;
    return ESP_OK;
}

// Coasts PWM, engages the hardware brake, and disables every motor.
esp_err_t drivetrain_brake(Drivetrain *drivetrain) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->status.initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t first_error = ESP_OK;
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        record_first_error(&first_error,
            motor_driver_coast(&drivetrain->devices.motors[index]));
    }
    const esp_err_t brake_error = gpio_set_level(
        (gpio_num_t)drivetrain->config->brake_pin, 1);
    record_first_error(&first_error, brake_error);
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        if (motor_driver_is_enabled(&drivetrain->devices.motors[index])) {
            record_first_error(&first_error,
                motor_driver_disable(&drivetrain->devices.motors[index]));
        }
    }

    drivetrain->status.enabled = false;
    drivetrain->status.brake_engaged = brake_error == ESP_OK;
    drivetrain->status.command_timeout_active = false;
    drivetrain->control.last_command_us = 0;
    drivetrain->control.last_update_us = 0;
    reset_control_state(drivetrain);
    return first_error;
}

// Commands zero PWM and releases the brake without disabling motor channels.
esp_err_t drivetrain_coast(Drivetrain *drivetrain) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->status.initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t first_error = ESP_OK;
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        record_first_error(&first_error,
            motor_driver_coast(&drivetrain->devices.motors[index]));
    }
    if (first_error != ESP_OK) {
        drivetrain_brake(drivetrain);
        return first_error;
    }

    esp_err_t error = gpio_set_level(
        (gpio_num_t)drivetrain->config->brake_pin, 0);
    if (error != ESP_OK) {
        drivetrain_brake(drivetrain);
        return error;
    }

    drivetrain->status.brake_engaged = false;
    reset_control_state(drivetrain);
    drivetrain->control.last_update_us = 0;
    return ESP_OK;
}

// Reads one encoder count while returning zero for invalid state or wheel ID.
int32_t drivetrain_get_encoder_accumulated_count(
    const Drivetrain *drivetrain,
    DrivetrainMotorId motor_id
) {
    if (drivetrain == NULL || !drivetrain->status.initialized ||
        !motor_id_is_valid(motor_id)) {
        return 0;
    }
    return drivetrain->devices.encoders[motor_id].accumulated_count;
}

// Reads one wheel's measured linear velocity in meters per second.
float drivetrain_get_encoder_velocity_mps(
    const Drivetrain *drivetrain,
    DrivetrainMotorId motor_id
) {
    if (drivetrain == NULL || !drivetrain->status.initialized ||
        !motor_id_is_valid(motor_id)) {
        return 0.0f;
    }
    return encoder_driver_get_velocity_mps(&drivetrain->devices.encoders[motor_id]);
}

// Reads one wheel's most recently calculated linear velocity target.
float drivetrain_get_target_velocity_mps(
    const Drivetrain *drivetrain,
    DrivetrainMotorId motor_id
) {
    if (drivetrain == NULL || !drivetrain->status.initialized ||
        !motor_id_is_valid(motor_id)) {
        return 0.0f;
    }
    return drivetrain->control.target_wheel_mps[motor_id];
}

// Reads one wheel's most recently applied signed duty cycle.
float drivetrain_get_applied_duty(
    const Drivetrain *drivetrain,
    DrivetrainMotorId motor_id
) {
    if (drivetrain == NULL || !drivetrain->status.initialized ||
        !motor_id_is_valid(motor_id)) {
        return 0.0f;
    }
    return drivetrain->control.last_duty[motor_id];
}
