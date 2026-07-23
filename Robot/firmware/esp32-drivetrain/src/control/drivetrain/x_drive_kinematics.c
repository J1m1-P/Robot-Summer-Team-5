/* Implements the pure X-drive body-to-wheel velocity Jacobian. */
#include "control/drivetrain/x_drive_kinematics.h"

#include <math.h>
#include <stdbool.h>
#include <stddef.h>

static bool config_is_valid(const XDriveKinematicsConfig *config)
{
    return config != NULL &&
           isfinite(config->wheel_radius_m) && config->wheel_radius_m > 0.0f &&
           isfinite(config->chassis_half_length_m) &&
           config->chassis_half_length_m >= 0.0f &&
           isfinite(config->chassis_half_width_m) &&
           config->chassis_half_width_m >= 0.0f &&
           isfinite(config->wheel_angle_rad);
}

// Converts a body-frame velocity into logical-order wheel angular velocities.
esp_err_t x_drive_kinematics_body_to_wheel_velocities(
    const XDriveKinematicsConfig *config,
    const DrivetrainBodyVelocity *body,
    XDriveWheelVelocity *wheels_out
) {
    if (!config_is_valid(config) || body == NULL || wheels_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!isfinite(body->vx) || !isfinite(body->vy) || !isfinite(body->omega)) {
        return ESP_ERR_INVALID_ARG;
    }

    const float cosine = cosf(config->wheel_angle_rad);
    const float sine = sinf(config->wheel_angle_rad);
    const float arm = config->chassis_half_length_m * sine +
                      config->chassis_half_width_m * cosine;
    const float radius = config->wheel_radius_m;

    /* Right-handed body frame: +x forward, +y left, +omega CCW. */
    wheels_out->fl = (cosine * body->vx - sine * body->vy - arm * body->omega) / radius;
    wheels_out->fr = (cosine * body->vx + sine * body->vy + arm * body->omega) / radius;
    wheels_out->bl = (cosine * body->vx + sine * body->vy - arm * body->omega) / radius;
    wheels_out->br = (cosine * body->vx - sine * body->vy + arm * body->omega) / radius;

    if (!isfinite(wheels_out->fl) || !isfinite(wheels_out->fr) ||
        !isfinite(wheels_out->bl) || !isfinite(wheels_out->br)) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

// Applies the inverse X-drive Jacobian to measured wheel angular velocities.
esp_err_t x_drive_kinematics_wheel_to_body_velocities(
    const XDriveKinematicsConfig *config,
    const XDriveWheelVelocity *wheels,
    DrivetrainBodyVelocity *body_out
) {
    if (!config_is_valid(config) || wheels == NULL || body_out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!isfinite(wheels->fl) || !isfinite(wheels->fr) ||
        !isfinite(wheels->bl) || !isfinite(wheels->br)) {
        return ESP_ERR_INVALID_ARG;
    }

    const float cosine = cosf(config->wheel_angle_rad);
    const float sine = sinf(config->wheel_angle_rad);
    const float arm = config->chassis_half_length_m * sine +
                      config->chassis_half_width_m * cosine;
    if (fabsf(cosine) <= 1.0e-6f || fabsf(sine) <= 1.0e-6f ||
        fabsf(arm) <= 1.0e-6f) {
        return ESP_ERR_INVALID_ARG;
    }

    const float radius = config->wheel_radius_m;
    body_out->vx = radius * (wheels->fl + wheels->fr + wheels->bl + wheels->br) /
                   (4.0f * cosine);
    body_out->vy = -radius * (wheels->fl - wheels->fr - wheels->bl + wheels->br) /
                   (4.0f * sine);
    body_out->omega = radius * (-wheels->fl + wheels->fr - wheels->bl + wheels->br) /
                      (4.0f * arm);

    return isfinite(body_out->vx) && isfinite(body_out->vy) &&
           isfinite(body_out->omega) ? ESP_OK : ESP_ERR_INVALID_ARG;
}
