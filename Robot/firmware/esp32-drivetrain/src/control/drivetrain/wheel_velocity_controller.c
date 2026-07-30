/* Implements one wheel's feedforward-plus-PI velocity controller. */
#include "control/drivetrain/wheel_velocity_controller.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include <robot_common/math_utils.h>

// Do not boost a wheel whose target is effectively zero. This matters for
// X-drive commands where one wheel can be near zero while the other wheels
// carry the requested translation/rotation vector.
static const float kBreakawayMinimumTargetMps = 0.02f;

// Validates every gain and limit used by the controller calculation.
bool wheel_velocity_controller_config_is_valid(const WheelVelocityControllerConfig *config) {
    return config != NULL &&
           isfinite(config->kff) && isfinite(config->kff_offset) &&
           isfinite(config->kp) && isfinite(config->ki) &&
           isfinite(config->output_min) && isfinite(config->output_max) &&
           config->output_min <= config->output_max &&
           isfinite(config->integral_min) && isfinite(config->integral_max) &&
           config->integral_min <= config->integral_max &&
           isfinite(config->duty_slew_per_s) && config->duty_slew_per_s >= 0.0f &&
           isfinite(config->breakaway_duty) && config->breakaway_duty >= 0.0f &&
           config->breakaway_duty <= config->output_max;
}

// Advances one feedforward-plus-PI controller by one time step.
esp_err_t wheel_velocity_controller_update(
    WheelVelocityController *controller,
    const WheelVelocityControllerConfig *config,
    float target_mps,
    float measured_mps,
    float dt_s,
    float *duty_out
) {
    if (controller == NULL || duty_out == NULL ||
        !wheel_velocity_controller_config_is_valid(config) ||
        !isfinite(controller->integral) || !isfinite(controller->last_duty) ||
        !isfinite(target_mps) || !isfinite(measured_mps) ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    const float error = target_mps - measured_mps;
    if (target_mps == 0.0f) {
        controller->integral = 0.0f;
    } else {
        controller->integral = clamp(
            controller->integral + error * dt_s,
            config->integral_min,
            config->integral_max);
    }

    float feedforward = config->kff * target_mps;
    if (target_mps > 0.0f) feedforward += config->kff_offset;
    if (target_mps < 0.0f) feedforward -= config->kff_offset;

    /* A stop command coasts rather than actively (reverse-polarity) braking:
     * plugging a spinning motor can draw more current than either running or
     * locked-rotor, since supply voltage and back-EMF add instead of oppose,
     * and this driver has no software current limiting to bound that. Forcing
     * output to exactly zero also satisfies the "don't keep driving forward
     * while slew catches up" goal the old sign-clamp existed for, without
     * ever commanding duty opposite the wheel's current motion. */
    float output = feedforward + config->kp * error + config->ki * controller->integral;
    if (target_mps == 0.0f) {
        output = 0.0f;
    }

    const float bounded = clamp(output, config->output_min, config->output_max);
    const float maximum_delta = config->duty_slew_per_s * dt_s;
    const float delta = bounded - controller->last_duty;
    if (delta > maximum_delta) {
        *duty_out = controller->last_duty + maximum_delta;
    } else if (delta < -maximum_delta) {
        *duty_out = controller->last_duty - maximum_delta;
    } else {
        *duty_out = bounded;
    }

    // A low-speed command can produce a valid but insufficient duty to
    // release a stationary wheel from static friction. Apply the assist only
    // on the first nonzero command after a true stop; subsequent cycles use
    // the normal PI controller so this does not become a speed offset.
    if (fabsf(target_mps) >= kBreakawayMinimumTargetMps &&
        controller->last_duty == 0.0f &&
        fabsf(measured_mps) < 0.02f && config->breakaway_duty > 0.0f) {
        const float breakaway = copysignf(config->breakaway_duty, target_mps);
        if (fabsf(*duty_out) < config->breakaway_duty) {
            *duty_out = breakaway;
        }
    }

    // A stop command overrides slew immediately rather than ramping to zero.
    if (target_mps == 0.0f) {
        *duty_out = 0.0f;
    }
    controller->last_duty = *duty_out;
    return ESP_OK;
}

// Clears integral history and the previous slew-limited output.
void wheel_velocity_controller_reset(WheelVelocityController *controller) {
    if (controller != NULL) memset(controller, 0, sizeof(*controller));
}
