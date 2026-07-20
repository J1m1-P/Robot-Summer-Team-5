/* Declares a jerk-bounded speed ramp toward a per-cycle target speed. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bounds how fast commanded acceleration itself may change (m/s^3), so a
 * motion primitive's step changes in target speed don't feed the
 * wheel-velocity PI a step change in acceleration either. This is a
 * simplified jerk-bounded ramp, not a full 7-segment S-curve: it rate-limits
 * acceleration toward whatever bang-bang acceleration would reach the
 * target speed, rather than pre-planning distinct accel/cruise/decel
 * segments. */
typedef struct {
    float max_jerk_mps3;
} SpeedProfileConfig;

// Rejects configurations that could produce undefined or unsafe behavior.
bool speed_profile_config_is_valid(const SpeedProfileConfig *config);

// Retains the last commanded speed and acceleration between cycles.
typedef struct {
    float commanded_speed_mps;
    float commanded_accel_mps2;
} SpeedProfile;

// Restores the ramp to a known starting speed with zero acceleration.
void speed_profile_reset(SpeedProfile *profile, float initial_speed_mps);

/* Advances the ramp by one control cycle toward `target_speed_mps`, bounding
 * both acceleration (`max_accel_mps2`) and its rate of change
 * (`config->max_jerk_mps3`). Never overshoots `target_speed_mps` in a single
 * call. */
esp_err_t speed_profile_update(
    SpeedProfile *profile,
    const SpeedProfileConfig *config,
    float target_speed_mps,
    float max_accel_mps2,
    float dt_s,
    float *commanded_speed_mps_out);

#ifdef __cplusplus
}
#endif
