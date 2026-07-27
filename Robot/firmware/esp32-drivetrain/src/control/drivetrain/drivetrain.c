/* Implements velocity control, hardware lifecycle, telemetry, and safe stopping. */
#include "control/drivetrain/drivetrain.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_timer.h"

#include <robot_common/app_log.h>
#include <robot_common/math_utils.h>

// Checks wheel identifiers before using them as array indexes.
static bool motor_id_is_valid(DrivetrainMotorId id) {
    return id >= DRIVETRAIN_MOTOR_FL && id < DRIVETRAIN_MOTOR_MAX;
}

// Preserves the first failure while allowing every cleanup action to run.
static void record_first_error(esp_err_t *first_error, esp_err_t error) {
    if (*first_error == ESP_OK && error != ESP_OK) *first_error = error;
}

// Clears velocity targets, PI history, and wheel-output telemetry.
static void reset_control_state(Drivetrain *drivetrain) {
    memset(&drivetrain->status.target_body, 0, sizeof(drivetrain->status.target_body));
    drivetrain->control.apply_motion_calibration = false;
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        wheel_velocity_controller_reset(&drivetrain->devices.wheel_controller[index]);
        drivetrain->control.target_wheel_mps[index] = 0.0f;
        drivetrain->control.last_duty[index] = 0.0f;
    }
}

// Checks PI output bounds against the final drivetrain duty limit.
static bool controller_config_fits_drivetrain(
    const WheelVelocityControllerConfig *controller_config,
    float max_duty
) {
    return wheel_velocity_controller_config_is_valid(controller_config) &&
           controller_config->output_min >= -max_duty &&
           controller_config->output_max <= max_duty;
}

// Validates one positive finite motion or timing bound.
static bool positive_finite(float value) {
    return isfinite(value) && value > 0.0f;
}

/* Uniformly scales a requested body velocity when its combined 30-degree
 * omniwheel demand would exceed the measured wheel-speed ceiling. Scaling all
 * components preserves the intended motion direction and prevents hidden
 * wheel-controller saturation. */
static esp_err_t limit_body_to_wheel_feasibility(
    const DrivetrainConfig *config,
    const DrivetrainBodyVelocity *requested,
    DrivetrainBodyVelocity *limited_out
) {
    if (config == NULL || requested == NULL || limited_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    XDriveWheelVelocity wheels = {0};
    const esp_err_t error = x_drive_kinematics_body_to_wheel_velocities(
        &config->x_drive_kinematics, requested, &wheels);
    if (error != ESP_OK) return error;
    const float radius = config->x_drive_kinematics.wheel_radius_m;
    const float peak_mps = fmaxf(fmaxf(fabsf(wheels.fl), fabsf(wheels.fr)),
                                 fmaxf(fabsf(wheels.bl), fabsf(wheels.br))) * radius;
    const float scale = peak_mps > config->max_wheel_speed_mps
        ? config->max_wheel_speed_mps / peak_mps
        : 1.0f;
    limited_out->vx = requested->vx * scale;
    limited_out->vy = requested->vy * scale;
    limited_out->omega = requested->omega * scale;
    return ESP_OK;
}

static MoveCalibrationDirection calibration_direction(
    float vx_mps, float vy_mps
) {
    if (fabsf(vx_mps) >= fabsf(vy_mps)) {
        return vx_mps >= 0.0f
            ? MOVE_CALIBRATION_POS_X
            : MOVE_CALIBRATION_NEG_X;
    }
    return vy_mps >= 0.0f
        ? MOVE_CALIBRATION_POS_Y
        : MOVE_CALIBRATION_NEG_Y;
}

static void apply_advanced_calibration(
    const Drivetrain *drivetrain, float wheel_mps[DRIVETRAIN_MOTOR_MAX]
) {
    const MoveCalibrationConfig *calibration = drivetrain->config->move_calibration;
    if (!drivetrain->control.apply_motion_calibration || calibration == NULL ||
        !calibration->enabled) return;

    if (hypotf(drivetrain->status.target_body.vx,
               drivetrain->status.target_body.vy) <= 1.0e-6f) return;
    const MoveCalibrationDirection direction = calibration_direction(
        drivetrain->status.target_body.vx, drivetrain->status.target_body.vy);
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        wheel_mps[index] *= calibration->f_lat[direction][index];
    }
    float peak = 0.0f;
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
        peak = fmaxf(peak, fabsf(wheel_mps[index]));
    }
    if (peak > drivetrain->config->max_wheel_speed_mps) {
        const float scale = drivetrain->config->max_wheel_speed_mps / peak;
        for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; ++index) {
            wheel_mps[index] *= scale;
        }
    }
}

// Validates hardware assignments, geometry, controller, and safety bounds.
static bool drivetrain_config_is_valid(const DrivetrainConfig *config) {
    if (config == NULL || !GPIO_IS_VALID_OUTPUT_GPIO(config->brake_pin)) return false;
    if (!positive_finite(config->max_duty) || config->max_duty > 1.0f ||
        !positive_finite(config->max_vx_mps) ||
        !positive_finite(config->max_vy_mps) ||
        !positive_finite(config->max_omega_rad_s) ||
        !positive_finite(config->max_wheel_speed_mps) ||
        !positive_finite(config->max_control_dt_s) ||
        config->command_timeout_us <= 0) {
        return false;
    }
    if (!controller_config_fits_drivetrain(&config->wheel_controller, config->max_duty)) return false;

    DrivetrainBodyVelocity zero_body = {0};
    XDriveWheelVelocity validation_output = {0};
    if (x_drive_kinematics_body_to_wheel_velocities(
            &config->x_drive_kinematics, &zero_body, &validation_output) != ESP_OK) {
        return false;
    }

    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
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
    for (int index = DRIVETRAIN_MOTOR_MAX - 1; index >= 0; index--) {
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
    for (int index = enabled_count - 1; index >= 0; index--) {
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
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
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
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        if (!isfinite(duties[index])) return ESP_ERR_INVALID_ARG;
        bounded[index] = clamp(
            duties[index],
            -drivetrain->config->max_duty,
            drivetrain->config->max_duty
        );
    }

    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
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

// Takes control of safety-critical outputs before any blocking startup work.
esp_err_t drivetrain_hold_safe_outputs(const DrivetrainConfig *config) {
    if (config == NULL || !GPIO_IS_VALID_OUTPUT_GPIO(config->brake_pin)) {
        return ESP_ERR_INVALID_ARG;
    }

    // Latch the active-high brake before switching the pin to output.
    esp_err_t error = gpio_set_level((gpio_num_t)config->brake_pin, 1);
    if (error != ESP_OK) return error;

    gpio_config_t brake_config = {0};
    brake_config.pin_bit_mask = 1ULL << config->brake_pin;
    brake_config.mode = GPIO_MODE_OUTPUT;
    brake_config.pull_up_en = GPIO_PULLUP_ENABLE;
    brake_config.pull_down_en = GPIO_PULLDOWN_DISABLE;
    brake_config.intr_type = GPIO_INTR_DISABLE;
    error = gpio_config(&brake_config);
    if (error != ESP_OK) return error;

    error = gpio_set_level((gpio_num_t)config->brake_pin, 1);
    if (error != ESP_OK) return error;

    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        error = motor_driver_hold_pwm_low(config->motor_configs[index]);
        if (error != ESP_OK) return error;
    }
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
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        drivetrain->control.active_controller_config[index] = config->wheel_controller;
    }

    esp_err_t error = drivetrain_hold_safe_outputs(config);
    if (error != ESP_OK) {
        memset(drivetrain, 0, sizeof(*drivetrain));
        return error;
    }
    drivetrain->status.brake_engaged = true;

    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
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
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        esp_err_t error = motor_driver_enable(&drivetrain->devices.motors[index]);
        if (error != ESP_OK) {
            cleanup_failed_enable(drivetrain, enabled_count);
            return error;
        }
        enabled_count++;
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

static esp_err_t set_body_velocity(
    Drivetrain *drivetrain,
    float vx_mps,
    float vy_mps,
    float omega_rad_s,
    bool advanced_motion
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

    DrivetrainBodyVelocity requested = {
        .vx = vx_mps,
        .vy = vy_mps,
        .omega = omega_rad_s,
    };
    /* The wheel-feasibility governor must bound the raw requested command
     * before calibration adjusts it (see TUNING_ROADMAP.md's advanced-motion
     * architecture diagram): world-to-body -> governor -> calibration
     * adapter. Applying F_lon first would let it push a command past the
     * ceiling that the governor is supposed to be bounding. */
    DrivetrainBodyVelocity limited = {0};
    const esp_err_t limit_error = limit_body_to_wheel_feasibility(
        drivetrain->config, &requested, &limited);
    if (limit_error != ESP_OK) return limit_error;

    const MoveCalibrationConfig *calibration = drivetrain->config->move_calibration;
    if (advanced_motion && calibration != NULL && calibration->enabled &&
        hypotf(limited.vx, limited.vy) > 1.0e-6f) {
        if (fabsf(limited.vx) > 1.0e-6f) {
            const MoveCalibrationDirection x_direction = limited.vx >= 0.0f
                ? MOVE_CALIBRATION_POS_X : MOVE_CALIBRATION_NEG_X;
            limited.vx *= calibration->f_lon[x_direction];
        }
        if (fabsf(limited.vy) > 1.0e-6f) {
            const MoveCalibrationDirection y_direction = limited.vy >= 0.0f
                ? MOVE_CALIBRATION_POS_Y : MOVE_CALIBRATION_NEG_Y;
            limited.vy *= calibration->f_lon[y_direction];
        }
    }
    drivetrain->status.target_body = limited;
    drivetrain->control.apply_motion_calibration = advanced_motion;
    drivetrain->control.last_command_us = esp_timer_get_time();
    drivetrain->status.command_timeout_active = false;
    return ESP_OK;
}

// Stores a bounded body target and refreshes the command watchdog.
esp_err_t drivetrain_set_body_velocity(
    Drivetrain *drivetrain, float vx_mps, float vy_mps, float omega_rad_s
) {
    return set_body_velocity(drivetrain, vx_mps, vy_mps, omega_rad_s, false);
}

esp_err_t drivetrain_set_advanced_body_velocity(
    Drivetrain *drivetrain, float vx_mps, float vy_mps, float omega_rad_s
) {
    return set_body_velocity(drivetrain, vx_mps, vy_mps, omega_rad_s, true);
}

// Immediately removes motor power while retaining a zero velocity target for
// subsequent control cycles.  This is deliberately a coast, not a reverse-
// polarity active brake: zero target is the motion API's completion contract,
// and a caller must not have to wait for another drivetrain_update() cycle
// before PWM is zero on every motor.
esp_err_t drivetrain_stop(Drivetrain *drivetrain) {
    const esp_err_t error = drivetrain_set_body_velocity(
        drivetrain, 0.0f, 0.0f, 0.0f);
    if (error != ESP_OK) return error;
    return drivetrain_coast(drivetrain);
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
        const esp_err_t coast_error = drivetrain_coast(drivetrain);
        return coast_error == ESP_OK ? ESP_ERR_INVALID_STATE : coast_error;
    }

    esp_err_t error = update_encoders(drivetrain);
    if (error != ESP_OK) return error;

    XDriveWheelVelocity wheel_rad_s = {0};
    error = x_drive_kinematics_body_to_wheel_velocities(
        &drivetrain->config->x_drive_kinematics,
        &drivetrain->status.target_body,
        &wheel_rad_s);
    if (error != ESP_OK) {
        drivetrain_brake(drivetrain);
        return error;
    }

    const float radius = drivetrain->config->x_drive_kinematics.wheel_radius_m;
    float wheel_velocities[] = {
        wheel_rad_s.fl * radius,
        wheel_rad_s.fr * radius,
        wheel_rad_s.bl * radius,
        wheel_rad_s.br * radius,
    };
    apply_advanced_calibration(drivetrain, wheel_velocities);
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        drivetrain->control.target_wheel_mps[index] = wheel_velocities[index];
    }

    float duties[DRIVETRAIN_MOTOR_MAX] = {0};
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        const float measured = encoder_driver_get_velocity_mps(
            &drivetrain->devices.encoders[index]);
        error = wheel_velocity_controller_update(
            &drivetrain->devices.wheel_controller[index],
            &drivetrain->control.active_controller_config[index],
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

// Copies the caller-facing status snapshot.
esp_err_t drivetrain_get_status(
    const Drivetrain *drivetrain,
    DrivetrainStatus *status_out
) {
    if (drivetrain == NULL || status_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->status.initialized) return ESP_ERR_INVALID_STATE;
    *status_out = drivetrain->status;
    return ESP_OK;
}

// Coasts PWM, engages the hardware brake, and disables every motor.
esp_err_t drivetrain_brake(Drivetrain *drivetrain) {
    if (drivetrain == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->status.initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t first_error = ESP_OK;
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        record_first_error(&first_error,
            motor_driver_coast(&drivetrain->devices.motors[index]));
    }
    const esp_err_t brake_error = gpio_set_level(
        (gpio_num_t)drivetrain->config->brake_pin, 1);
    record_first_error(&first_error, brake_error);
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
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
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
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

// -----------------------------------------------------------------------------
// Real-time tuning only
// These functions mutate or inspect RAM-only controller configuration.
// -----------------------------------------------------------------------------

// Validates and installs live tuning gains, then clears prior controller history.
esp_err_t drivetrain_set_wheel_controller_config(
    Drivetrain *drivetrain,
    const WheelVelocityControllerConfig *config
) {
    if (drivetrain == NULL || config == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->status.initialized) return ESP_ERR_INVALID_STATE;
    if (!controller_config_fits_drivetrain(config, drivetrain->config->max_duty)) {
        return ESP_ERR_INVALID_ARG;
    }
    for (int index = 0; index < DRIVETRAIN_MOTOR_MAX; index++) {
        drivetrain->control.active_controller_config[index] = *config;
        wheel_velocity_controller_reset(&drivetrain->devices.wheel_controller[index]);
    }
    return ESP_OK;
}

// Copies the live gains without exposing mutable controller ownership.
esp_err_t drivetrain_get_wheel_controller_config(
    const Drivetrain *drivetrain,
    WheelVelocityControllerConfig *config_out
) {
    if (drivetrain == NULL || config_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!drivetrain->status.initialized) return ESP_ERR_INVALID_STATE;
    *config_out = drivetrain->control.active_controller_config[DRIVETRAIN_MOTOR_FL];
    return ESP_OK;
}

esp_err_t drivetrain_set_motor_controller_config(
    Drivetrain *drivetrain,
    DrivetrainMotorId motor_id,
    const WheelVelocityControllerConfig *config
) {
    if (drivetrain == NULL || config == NULL || !motor_id_is_valid(motor_id)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!drivetrain->status.initialized) return ESP_ERR_INVALID_STATE;
    if (!controller_config_fits_drivetrain(config, drivetrain->config->max_duty)) {
        return ESP_ERR_INVALID_ARG;
    }
    drivetrain->control.active_controller_config[motor_id] = *config;
    wheel_velocity_controller_reset(&drivetrain->devices.wheel_controller[motor_id]);
    return ESP_OK;
}

esp_err_t drivetrain_get_motor_controller_config(
    const Drivetrain *drivetrain,
    DrivetrainMotorId motor_id,
    WheelVelocityControllerConfig *config_out
) {
    if (drivetrain == NULL || config_out == NULL || !motor_id_is_valid(motor_id)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!drivetrain->status.initialized) return ESP_ERR_INVALID_STATE;
    *config_out = drivetrain->control.active_controller_config[motor_id];
    return ESP_OK;
}
