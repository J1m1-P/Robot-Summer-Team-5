/* Implements RotS: an open-loop, dead-reckoned in-place rotation. */
#include "control/drivetrain/rot_s.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

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

    const bool within_tolerance = fabsf(remaining_rad) <= rot->config->angle_tolerance_rad;
    const float target_omega_rad_s = within_tolerance
        ? 0.0f
        : copysignf(
              fminf(rot->max_omega_rad_s, sqrtf(fmaxf(0.0f, 2.0f * rot->config->max_alpha_rad_s2 *
                  (fabsf(remaining_rad) - rot->config->angle_tolerance_rad)))),
              remaining_rad);

    float commanded_omega_rad_s = 0.0f;
    const esp_err_t error = speed_profile_update(
        &rot->profile, &rot->config->speed_profile, target_omega_rad_s,
        rot->config->max_alpha_rad_s2, dt_s, &commanded_omega_rad_s);
    if (error != ESP_OK) {
        return error;
    }

    rot->planned_progress_rad += commanded_omega_rad_s * dt_s;

    output->requested_velocity.vx = 0.0f;
    output->requested_velocity.vy = 0.0f;
    output->requested_velocity.omega = commanded_omega_rad_s;

    if (within_tolerance && fabsf(commanded_omega_rad_s) <= kStoppedOmegaRadS) {
        rot->status = ROT_S_COMPLETE;
    }
    output->status = rot->status;
    output->motion_valid = true;
    return ESP_OK;
}
