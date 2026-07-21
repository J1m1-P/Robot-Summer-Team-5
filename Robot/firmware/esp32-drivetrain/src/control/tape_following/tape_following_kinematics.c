/* Implements the pure tape-following velocity-to-turn-rate mapping. */
#include "control/tape_following/tape_following_kinematics.h"

#include <math.h>
#include <stddef.h>

#include <robot_common/math_utils.h>

bool tape_following_kinematics_config_is_valid(
    const TapeFollowingKinematicsConfig *config)
{
    return config != NULL &&
           isfinite(config->gain_s_inv) && config->gain_s_inv >= 0.0f &&
           isfinite(config->max_omega_rad_s) &&
           config->max_omega_rad_s >= 0.0f &&
           isfinite(config->max_acceleration_rad_s2) &&
           config->max_acceleration_rad_s2 > 0.0f;
}

esp_err_t tape_following_kinematics_velocity_to_angular_velocity(
    const TapeFollowingKinematicsConfig *config,
    float travel_velocity_mps,
    float lateral_velocity_mps,
    float previous_omega_rad_s,
    float dt_s,
    float *omega_out)
{
    if (!tape_following_kinematics_config_is_valid(config) || omega_out == NULL ||
        !isfinite(travel_velocity_mps) || travel_velocity_mps == 0.0f ||
        !isfinite(lateral_velocity_mps) || !isfinite(previous_omega_rad_s) ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    const float direction = travel_velocity_mps > 0.0f ? 1.0f : -1.0f;
    const float travel_angle_rad = atan2f(
        lateral_velocity_mps, fabsf(travel_velocity_mps));
    const float target_omega_rad_s = clamp(
        -direction * config->gain_s_inv * travel_angle_rad,
        -config->max_omega_rad_s,
        config->max_omega_rad_s);
    const float maximum_change_rad_s = config->max_acceleration_rad_s2 * dt_s;

    *omega_out = clamp(
        target_omega_rad_s,
        previous_omega_rad_s - maximum_change_rad_s,
        previous_omega_rad_s + maximum_change_rad_s);
    return ESP_OK;
}
