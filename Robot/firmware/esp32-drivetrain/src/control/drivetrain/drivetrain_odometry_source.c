/* Implements the encoder-tick to world-frame odometry bridge. */
#include "control/drivetrain/drivetrain_odometry_source.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Corrects lateral (body +y) encoder-derived distance for measured wheel
 * slip during sideways X-drive motion: bench testing (PY move, compared
 * against PMW3610 optical ground truth) showed the raw encoder/kinematics
 * estimate over-reports true lateral travel by a factor of 1.308x, so the
 * correction is the reciprocal, not 1.308 itself. Forward (body +x) motion
 * showed no comparable slip and is left unscaled. Global because this is
 * the single encoder-tick-to-body-delta bridge every consumer (PoseTracker's
 * encoder fallback, and any future direct caller such as MoveL/MoveP/MoveC)
 * goes through -- there is no separate copy of this conversion anywhere. */
#define LATERAL_ENCODER_SLIP_RATIO 1.308f
#define LATERAL_ENCODER_SCALE (1.0f / LATERAL_ENCODER_SLIP_RATIO)

/* x_drive_kinematics.h exposes no public config validity check of its own,
 * so geometry problems (e.g. zero wheel radius) surface instead as a failed
 * x_drive_kinematics_wheel_to_body_velocities() call each cycle, which this
 * module reports as an invalid odometry cycle rather than a hard error. */
bool drivetrain_odometry_source_config_is_valid(
    const DrivetrainOdometrySourceConfig *config)
{
    return config != NULL &&
           config->counts_per_revolution_fl > 0 &&
           config->counts_per_revolution_fr > 0 &&
           config->counts_per_revolution_bl > 0 &&
           config->counts_per_revolution_br > 0;
}

void drivetrain_odometry_source_reset(DrivetrainOdometrySource *source)
{
    if (source != NULL) memset(source, 0, sizeof(*source));
}

esp_err_t drivetrain_odometry_source_compute_delta(
    DrivetrainOdometrySource *source,
    const DrivetrainOdometrySourceConfig *config,
    const DrivetrainWheelCounts *counts,
    DrivetrainOdometryDelta *delta_out,
    bool *has_delta_out,
    bool *valid_out)
{
    if (source == NULL || counts == NULL || delta_out == NULL ||
        has_delta_out == NULL || valid_out == NULL ||
        !drivetrain_odometry_source_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!source->has_previous_counts) {
        source->previous_counts = *counts;
        source->has_previous_counts = true;
        *has_delta_out = false;
        return ESP_OK;
    }

    /* Ticks -> wheel-angle delta (rad) since the previous cycle. Feeding a
     * position delta into the velocity Jacobian (rather than a rate) yields
     * a body-frame position delta instead of a velocity -- the same reuse
     * the diagnostic harness's get_relative_body_motion() prototype relies on. */
    const XDriveWheelVelocity wheel_angle_delta_rad = {
        .fl = (float)(counts->fl - source->previous_counts.fl) *
              (2.0f * (float)M_PI) / (float)config->counts_per_revolution_fl,
        .fr = (float)(counts->fr - source->previous_counts.fr) *
              (2.0f * (float)M_PI) / (float)config->counts_per_revolution_fr,
        .bl = (float)(counts->bl - source->previous_counts.bl) *
              (2.0f * (float)M_PI) / (float)config->counts_per_revolution_bl,
        .br = (float)(counts->br - source->previous_counts.br) *
              (2.0f * (float)M_PI) / (float)config->counts_per_revolution_br,
    };
    source->previous_counts = *counts;

    DrivetrainBodyVelocity body_delta_m_rad = {};
    const esp_err_t kinematics_error = x_drive_kinematics_wheel_to_body_velocities(
        &config->x_drive_kinematics, &wheel_angle_delta_rad, &body_delta_m_rad);

    *delta_out = (DrivetrainOdometryDelta){
        .forward_mm = body_delta_m_rad.vx * 1000.0f,
        .lateral_mm = body_delta_m_rad.vy * 1000.0f * LATERAL_ENCODER_SCALE,
        .heading_delta_rad = body_delta_m_rad.omega,
    };
    *has_delta_out = true;
    *valid_out = kinematics_error == ESP_OK;
    return ESP_OK;
}

esp_err_t drivetrain_odometry_source_update(
    DrivetrainOdometrySource *source,
    const DrivetrainOdometrySourceConfig *config,
    const DrivetrainWheelCounts *counts,
    DrivetrainOdometry *odometry)
{
    if (odometry == NULL) return ESP_ERR_INVALID_ARG;

    DrivetrainOdometryDelta delta = {0};
    bool has_delta = false;
    bool valid = false;
    const esp_err_t error = drivetrain_odometry_source_compute_delta(
        source, config, counts, &delta, &has_delta, &valid);
    if (error != ESP_OK || !has_delta) return error;

    return drivetrain_odometry_update(odometry, &delta, valid);
}
