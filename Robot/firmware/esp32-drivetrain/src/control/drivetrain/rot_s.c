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
           config->angle_tolerance_rad >= 0.0f;
}

esp_err_t rot_s_start(
    RotS *rot,
    const RotSConfig *config,
    float start_heading_rad,
    float angle_rad,
    float max_omega_rad_s,
    float max_alpha_rad_s2)
{
    if (rot == NULL || !rot_s_config_is_valid(config) ||
        !isfinite(start_heading_rad) ||
        !isfinite(angle_rad) || angle_rad == 0.0f ||
        !isfinite(max_omega_rad_s) || max_omega_rad_s <= 0.0f ||
        !isfinite(max_alpha_rad_s2) || max_alpha_rad_s2 <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(rot, 0, sizeof(*rot));
    rot->config = config;
    rot->start_heading_rad = start_heading_rad;
    rot->angle_rad = angle_rad;
    rot->max_omega_rad_s = max_omega_rad_s;
    rot->max_alpha_rad_s2 = max_alpha_rad_s2;
    speed_profile_reset(&rot->profile, 0.0f);
    rot->status = ROT_S_RUNNING;
    return ESP_OK;
}

esp_err_t rot_s_update(
    RotS *rot,
    float current_heading_rad,
    float dt_s,
    RotSOutput *output)
{
    if (rot == NULL || output == NULL ||
        !isfinite(current_heading_rad) ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (rot->config == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    memset(output, 0, sizeof(*output));

    /* Progress against the heading captured at start. DrivetrainPose's
     * heading_rad accumulates continuously (never wraps), so this plain
     * subtraction is exact -- no unwrap logic needed, same as MoveS's
     * position progress. */
    const float progress_rad = current_heading_rad - rot->start_heading_rad;
    const float remaining_rad = rot->angle_rad - progress_rad;
    output->remaining_angle_rad = remaining_rad;

    const float direction = rot->angle_rad >= 0.0f ? 1.0f : -1.0f;
    const bool within_tolerance = fabsf(remaining_rad) <= rot->config->angle_tolerance_rad;
    const float target_omega_rad_s = within_tolerance
        ? 0.0f
        : direction * fminf(rot->max_omega_rad_s, sqrtf(fmaxf(0.0f, 2.0f * rot->max_alpha_rad_s2 *
                 (fabsf(remaining_rad) - rot->config->angle_tolerance_rad))));

    float commanded_omega_rad_s = 0.0f;
    const esp_err_t error = speed_profile_update(
        &rot->profile, &rot->config->speed_profile, target_omega_rad_s,
        rot->max_alpha_rad_s2, dt_s, &commanded_omega_rad_s);
    if (error != ESP_OK) {
        return error;
    }

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
