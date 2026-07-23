/* Declares the pure body-velocity to X-drive wheel-velocity transform. */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Represents a commanded velocity in the robot body frame.
typedef struct {
    float vx;     // m/s, positive forward
    float vy;     // m/s, positive strafe left
    float omega;  // rad/s, positive counterclockwise
} DrivetrainBodyVelocity;

// Represents logical-order wheel angular velocities in radians per second.
typedef struct {
    float fl;
    float fr;
    float bl;
    float br;
} XDriveWheelVelocity;

// Defines the physical geometry used by the X-drive velocity Jacobian.
typedef struct {
    float wheel_radius_m;
    float chassis_half_length_m;
    float chassis_half_width_m;
    float wheel_angle_rad;
} XDriveKinematicsConfig;

// Converts a body-frame velocity into FL/FR/BL/BR wheel angular velocities.
esp_err_t x_drive_kinematics_body_to_wheel_velocities(
    const XDriveKinematicsConfig *config,
    const DrivetrainBodyVelocity *body,
    XDriveWheelVelocity *wheels_out
);

// Converts FL/FR/BL/BR wheel angular velocities back to body-frame velocity.
esp_err_t x_drive_kinematics_wheel_to_body_velocities(
    const XDriveKinematicsConfig *config,
    const XDriveWheelVelocity *wheels,
    DrivetrainBodyVelocity *body_out
);

#ifdef __cplusplus
}
#endif
