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

/* Predicts how far the current profile will travel if every subsequent
 * update targets zero speed.  The simulation uses the same jerk-limited
 * update rule as speed_profile_update(), so it includes the profile's
 * current acceleration as well as max_accel_mps2 and max_jerk_mps3.
 *
 * `dt_s` is used as the simulation's internal step size, same as
 * speed_profile_update(), but clamped to a small ceiling first: a well-
 * behaved caller's dt_s (a normal control period) passes through unchanged,
 * but an anomalously large one is capped rather than fed straight into the
 * simulation. Reusing a live dt_s uncapped directly biased the predicted
 * distance on real hardware: desired_accel_mps2 in the underlying update is
 * speed_error/dt_s, so one slow real cycle (e.g. a blocking Serial print)
 * inflated the predicted stopping distance, and because braking is a one-way
 * commitment (never re-evaluated once triggered), a single glitchy cycle
 * could permanently lock a move into braking far too early.
 *
 * The result is always a non-negative magnitude and is unit-agnostic: it is
 * metres for a linear profile and radians for RotS's angular profile.
 * `stopped_speed_mps` is the caller's zero-speed threshold. */
esp_err_t speed_profile_predict_stopping_distance(
    const SpeedProfile *profile,
    const SpeedProfileConfig *config,
    float max_accel_mps2,
    float dt_s,
    float stopped_speed_mps,
    float *stopping_distance_m_out);

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
