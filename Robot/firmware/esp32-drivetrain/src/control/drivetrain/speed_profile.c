/* Implements a jerk-bounded speed ramp toward a per-cycle target speed. */
#include "control/drivetrain/speed_profile.h"

#include <math.h>
#include <stddef.h>

#include <robot_common/math_utils.h>

bool speed_profile_config_is_valid(const SpeedProfileConfig *config)
{
    return config != NULL &&
           isfinite(config->max_jerk_mps3) && config->max_jerk_mps3 > 0.0f;
}

void speed_profile_reset(SpeedProfile *profile, float initial_speed_mps)
{
    if (profile == NULL) return;
    profile->commanded_speed_mps = initial_speed_mps;
    profile->commanded_accel_mps2 = 0.0f;
}

esp_err_t speed_profile_update(
    SpeedProfile *profile,
    const SpeedProfileConfig *config,
    float target_speed_mps,
    float max_accel_mps2,
    float dt_s,
    float *commanded_speed_mps_out)
{
    if (profile == NULL || commanded_speed_mps_out == NULL ||
        !speed_profile_config_is_valid(config) ||
        !isfinite(target_speed_mps) ||
        !isfinite(max_accel_mps2) || max_accel_mps2 <= 0.0f ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Bang-bang acceleration that would land exactly on the target speed
     * this cycle if it could be commanded instantly, bounded by max_accel. */
    const float speed_error = target_speed_mps - profile->commanded_speed_mps;
    const float desired_accel_mps2 =
        clamp(speed_error / dt_s, -max_accel_mps2, max_accel_mps2);

    /* Rate-limit acceleration itself toward that bang-bang value. */
    const float max_accel_step = config->max_jerk_mps3 * dt_s;
    const float accel_step = clamp(
        desired_accel_mps2 - profile->commanded_accel_mps2,
        -max_accel_step, max_accel_step);
    float new_accel_mps2 = clamp(
        profile->commanded_accel_mps2 + accel_step,
        -max_accel_mps2, max_accel_mps2);

    float new_speed_mps = profile->commanded_speed_mps + new_accel_mps2 * dt_s;

    /* The jerk-limited accel can still overshoot on the final approach (this
     * is the "simplified" trade-off vs. a full 7-segment S-curve) -- clamp
     * to the target and zero acceleration rather than oscillate around it. */
    const bool overshot = (speed_error > 0.0f && new_speed_mps > target_speed_mps) ||
                          (speed_error < 0.0f && new_speed_mps < target_speed_mps);
    if (overshot) {
        new_speed_mps = target_speed_mps;
        new_accel_mps2 = 0.0f;
    }

    profile->commanded_speed_mps = new_speed_mps;
    profile->commanded_accel_mps2 = new_accel_mps2;
    *commanded_speed_mps_out = new_speed_mps;
    return ESP_OK;
}
