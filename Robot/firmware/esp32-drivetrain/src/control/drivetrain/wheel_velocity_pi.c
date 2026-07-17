/* Implements one wheel's feedforward-plus-PI velocity controller. */
#include "control/drivetrain/wheel_velocity_pi.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// Bounds a scalar to an inclusive minimum and maximum.
static float clamp_value(float value, float minimum, float maximum) {
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

// Validates every gain and limit used by the controller calculation.
bool wheel_velocity_pi_config_is_valid(const WheelVelocityPiConfig *config) {
    return config != NULL &&
           isfinite(config->kff) && isfinite(config->kff_offset) &&
           isfinite(config->kp) && isfinite(config->ki) &&
           isfinite(config->output_min) && isfinite(config->output_max) &&
           config->output_min <= config->output_max &&
           isfinite(config->integral_min) && isfinite(config->integral_max) &&
           config->integral_min <= config->integral_max &&
           isfinite(config->duty_slew_per_s) && config->duty_slew_per_s >= 0.0f;
}

// Advances one feedforward-plus-PI controller by one time step.
esp_err_t wheel_velocity_pi_update(
    WheelVelocityPi *pi,
    const WheelVelocityPiConfig *config,
    float target_mps,
    float measured_mps,
    float dt_s,
    float *duty_out
) {
    if (pi == NULL || duty_out == NULL ||
        !wheel_velocity_pi_config_is_valid(config) ||
        !isfinite(target_mps) || !isfinite(measured_mps) ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    const float error = target_mps - measured_mps;
    if (target_mps == 0.0f) {
        pi->integral = 0.0f;
    } else {
        pi->integral = clamp_value(
            pi->integral + error * dt_s,
            config->integral_min,
            config->integral_max);
    }

    float feedforward = config->kff * target_mps;
    if (target_mps > 0.0f) feedforward += config->kff_offset;
    if (target_mps < 0.0f) feedforward -= config->kff_offset;

    float output = feedforward + config->kp * error + config->ki * pi->integral;
    if (target_mps == 0.0f) {
        if (measured_mps > 0.0f) output = fminf(output, 0.0f);
        if (measured_mps < 0.0f) output = fmaxf(output, 0.0f);
    }

    const float bounded = clamp_value(output, config->output_min, config->output_max);
    const float maximum_delta = config->duty_slew_per_s * dt_s;
    const float delta = bounded - pi->last_duty;
    if (delta > maximum_delta) {
        *duty_out = pi->last_duty + maximum_delta;
    } else if (delta < -maximum_delta) {
        *duty_out = pi->last_duty - maximum_delta;
    } else {
        *duty_out = bounded;
    }

    // A stop command overrides slew if slew would continue driving with motion.
    if (target_mps == 0.0f) {
        if (measured_mps > 0.0f) *duty_out = fminf(*duty_out, 0.0f);
        if (measured_mps < 0.0f) *duty_out = fmaxf(*duty_out, 0.0f);
    }
    pi->last_duty = *duty_out;
    return ESP_OK;
}

// Clears integral history and the previous slew-limited output.
void wheel_velocity_pi_reset(WheelVelocityPi *pi) {
    if (pi != NULL) memset(pi, 0, sizeof(*pi));
}
