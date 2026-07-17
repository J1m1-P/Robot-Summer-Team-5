/* Declares the pure body-velocity to X-drive wheel-velocity transform. */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Represents a commanded velocity in the robot body frame.
typedef struct {
    float vx;     // m/s, positive forward
    float vy;     // m/s, positive strafe right
    float omega;  // rad/s, positive counterclockwise
} DrivetrainBodyVelocity;

// Represents logical-order wheel angular velocities in radians per second.
typedef struct {
    float fl;
    float fr;
    float bl;
    float br;
} DrivetrainWheelVelocity;

// Defines the physical geometry used by the X-drive velocity Jacobian.
typedef struct {
    float wheel_radius_m;
    float chassis_half_length_m;
    float chassis_half_width_m;
    float wheel_angle_rad;
} DrivetrainVelocityKinematicsConfig;

// Converts a body-frame velocity into FL/FR/BL/BR wheel angular velocities.
esp_err_t drivetrain_kinematics_body_to_wheel_velocities(
    const DrivetrainVelocityKinematicsConfig *config,
    const DrivetrainBodyVelocity *body,
    DrivetrainWheelVelocity *wheels_out
);

// Converts FL/FR/BL/BR wheel angular velocities back to body-frame velocity.
esp_err_t drivetrain_kinematics_wheel_to_body_velocities(
    const DrivetrainVelocityKinematicsConfig *config,
    const DrivetrainWheelVelocity *wheels,
    DrivetrainBodyVelocity *body_out
);

#ifdef __cplusplus
}
#endif
