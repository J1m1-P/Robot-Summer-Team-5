/* Implements RotS: an open-loop, dead-reckoned in-place rotation. */
#include "control/drivetrain/rot_s.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include <robot_common/math_utils.h>

/* Below this commanded angular speed, the jerk-bounded ramp is considered
 * to have actually reached zero rather than just asymptotically approaching it. */
static const float kStoppedOmegaRadS = 0.005f;

bool rot_s_config_is_valid(const RotSConfig *config)
{
    return config != NULL &&
           speed_profile_config_is_valid(&config->speed_profile) &&
           isfinite(config->angle_tolerance_rad) &&
           config->angle_tolerance_rad >= 0.0f &&
           isfinite(config->max_alpha_rad_s2) && config->max_alpha_rad_s2 > 0.0f;
}

esp_err_t rot_s_start(
    RotS *rot,
    const RotSConfig *config,
    float angle_rad,
    float max_omega_rad_s)
{
    if (rot == NULL || !rot_s_config_is_valid(config) ||
        !isfinite(angle_rad) || angle_rad == 0.0f ||
        !isfinite(max_omega_rad_s) || max_omega_rad_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(rot, 0, sizeof(*rot));
    rot->config = config;
    rot->angle_rad = angle_rad;
    rot->max_omega_rad_s = max_omega_rad_s;
    speed_profile_reset(&rot->profile, 0.0f);
    rot->status = ROT_S_RUNNING;
    return ESP_OK;
}

esp_err_t rot_s_update(
    RotS *rot,
    float dt_s,
    RotSOutput *output)
{
    if (rot == NULL || output == NULL ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (rot->config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(output, 0, sizeof(*output));

    /* Remaining angle against this rotation's OWN self-integrated progress
     * -- never against real heading. See rot_s.h's header comment: reading
     * real odometry here would let genuine mechanical error quietly change
     * how long the rotation runs instead of showing up as measurable angle
     * error, defeating the calibration this primitive exists to feed. Since
     * planned_progress_rad only ever moves in response to THIS module's own
     * commanded omega (which speed_profile_update() guarantees converges
     * cleanly to whatever target it's given), the runaway-in-one-direction
     * failure mode a real sensor's lag/noise could previously trigger is no
     * longer reachable -- there is no external signal left to disagree with
     * the plan. copysignf is kept anyway as cheap, correct insurance. */
    const float remaining_rad = rot->angle_rad - rot->planned_progress_rad;
    output->remaining_angle_rad = remaining_rad;

    /* A calibration rotation is a finite, one-way experiment.  In
     * particular, do not let a self-integrated overshoot change the target
     * sign: that would turn this open-loop maneuver into an oscillator.
     * Simulating the existing jerk-limited profile to zero gives a braking
     * distance consistent with its current acceleration state. */
    if (!rot->braking) {
        float stopping_distance_rad = 0.0f;
        const esp_err_t error = speed_profile_predict_stopping_distance(
            &rot->profile, &rot->config->speed_profile,
            rot->config->max_alpha_rad_s2, dt_s, kStoppedOmegaRadS,
            &stopping_distance_rad);
        if (error != ESP_OK) return error;
        if (fabsf(remaining_rad) <= stopping_distance_rad + rot->config->angle_tolerance_rad) {
            rot->braking = true;
        }
    }
    const float target_omega_rad_s = rot->braking
        ? 0.0f
        : copysignf(rot->max_omega_rad_s, rot->angle_rad);

    float commanded_omega_rad_s = 0.0f;
    const esp_err_t error = speed_profile_update(
        &rot->profile, &rot->config->speed_profile, target_omega_rad_s,
        rot->config->max_alpha_rad_s2, dt_s, &commanded_omega_rad_s);
    if (error != ESP_OK) {
        return error;
    }

    /* Defense in depth: speed_profile_update()'s overshoot-clamp only
     * guarantees its output won't cross THIS cycle's target -- it is not an
     * absolute ceiling. If target_omega_rad_s itself is changing cycle to
     * cycle (as it does right where deceleration begins) faster than that
     * relative check can track, nothing else stops commanded_omega_rad_s
     * from drifting past max_omega_rad_s -- observed on hardware as it
     * climbing to nearly 3x the configured ceiling and never recovering.
     * Clamp explicitly, and write the clamped value back into the profile's
     * own state so a bad step can't leave phantom "excess" speed baked into
     * next cycle's calculation. */
    commanded_omega_rad_s = clamp(commanded_omega_rad_s, -rot->max_omega_rad_s, rot->max_omega_rad_s);
    rot->profile.commanded_speed_mps = commanded_omega_rad_s;

    rot->planned_progress_rad += commanded_omega_rad_s * dt_s;

    output->requested_velocity.vx = 0.0f;
    output->requested_velocity.vy = 0.0f;
    output->requested_velocity.omega = commanded_omega_rad_s;

    if (rot->braking && fabsf(commanded_omega_rad_s) <= kStoppedOmegaRadS) {
        rot->status = ROT_S_COMPLETE;
        /* As with MoveS, a complete status must never be paired with the
         * profile's final sub-threshold angular-velocity sample. */
        output->requested_velocity.vx = 0.0f;
        output->requested_velocity.vy = 0.0f;
        output->requested_velocity.omega = 0.0f;
    }
    output->status = rot->status;
    output->motion_valid = true;
    return ESP_OK;
}
