#include "control/drivetrain_velocity_kinematics.h"

#include <cmath>

esp_err_t drivetrain_kinematics_body_to_wheel_velocities(
    const DrivetrainVelocityKinematicsConfig &config,
    const DrivetrainBodyVelocity &body,
    DrivetrainWheelVelocity &wheels_out
) {
    if (!std::isfinite(body.vx) || !std::isfinite(body.vy) || !std::isfinite(body.omega)) return ESP_ERR_INVALID_ARG;
    if (!std::isfinite(config.wheel_radius_m) || config.wheel_radius_m <= 0.0f) return ESP_ERR_INVALID_ARG;
    if (!std::isfinite(config.wheel_angle_rad)) return ESP_ERR_INVALID_ARG;

    // Corner signs follow the same FL/FR/BL/BR mirroring convention as
    // drivetrain_kinematics.c's duty mixer -- verify against actual wheel
    // mounting/motor wiring before trusting the signs.
    const float cb = std::cos(config.wheel_angle_rad);
    const float sb = std::sin(config.wheel_angle_rad);
    const float arm = config.chassis_half_length_m * sb + config.chassis_half_width_m * cb;

    wheels_out.fl = (cb * body.vx + sb * body.vy - arm * body.omega) / config.wheel_radius_m;
    wheels_out.fr = (cb * body.vx - sb * body.vy + arm * body.omega) / config.wheel_radius_m;
    wheels_out.bl = (cb * body.vx - sb * body.vy - arm * body.omega) / config.wheel_radius_m;
    wheels_out.br = (cb * body.vx + sb * body.vy + arm * body.omega) / config.wheel_radius_m;

    return ESP_OK;
}
