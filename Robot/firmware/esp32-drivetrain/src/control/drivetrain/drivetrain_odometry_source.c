/* Implements the encoder-tick to world-frame odometry bridge. */
#include "control/drivetrain/drivetrain_odometry_source.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

esp_err_t drivetrain_odometry_source_update(
    DrivetrainOdometrySource *source,
    const DrivetrainOdometrySourceConfig *config,
    const DrivetrainWheelCounts *counts,
    DrivetrainOdometry *odometry)
{
    if (source == NULL || counts == NULL || odometry == NULL ||
        !drivetrain_odometry_source_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!source->has_previous_counts) {
        source->previous_counts = *counts;
        source->has_previous_counts = true;
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

    const DrivetrainOdometryDelta delta = {
        .forward_mm = body_delta_m_rad.vx * 1000.0f,
        .lateral_mm = body_delta_m_rad.vy * 1000.0f,
        .heading_delta_rad = body_delta_m_rad.omega,
    };
    return drivetrain_odometry_update(odometry, &delta, kinematics_error == ESP_OK);
}
