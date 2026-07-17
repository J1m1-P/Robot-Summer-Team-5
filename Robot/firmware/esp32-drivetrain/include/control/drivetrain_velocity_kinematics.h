#pragma once

#include "esp_err.h"

// Velocity-level (Jacobian) inverse kinematics for the 4x omni-wheel
// X-drive: all four wheels mounted at the same roller angle magnitude
// (beta = 30 deg, matching WHEEL_ANGLE in config/drivetrain_config.c),
// mirrored per corner. Per wheel:
//
//   phi_dot_i = (1/r) * [cos(beta), sin(beta), -(l*sin(beta) + w*cos(beta))] . [vx, vy, omega]
//
// with signs on the vy and omega terms flipped per corner (see the TODO
// in the .cpp file). l is chassis half-length (center to axle, front-back),
// w is chassis half-width (center to wheel, left-right), r is wheel
// radius.
//
// This is a separate, not-yet-wired-up model living alongside
// control/drivetrain_kinematics.h (the existing open-loop duty mixer used
// by control/drivetrain.c, still C). Nothing here is called from
// production code yet.

struct DrivetrainBodyVelocity {
    float vx = 0.0f;     // m/s, +forward
    float vy = 0.0f;     // m/s, +strafe right
    float omega = 0.0f;  // rad/s, +CCW
};

struct DrivetrainWheelVelocity {
    float fl = 0.0f;  // rad/s
    float fr = 0.0f;
    float bl = 0.0f;
    float br = 0.0f;
};

struct DrivetrainVelocityKinematicsConfig {
    float wheel_radius_m = 0.0f;        // r
    float chassis_half_length_m = 0.0f; // l
    float chassis_half_width_m = 0.0f;  // w
    float wheel_angle_rad = 0.0f;       // beta, shared magnitude across all 4 wheels (X-drive)
};

// Takes config/body by const reference (required, always valid -- no null
// check needed) and writes the result into wheels_out by reference,
// avoiding a copy of the small-but-not-trivial-to-return-atomically
// 4-float result while still reading naturally at the call site:
//
//   DrivetrainWheelVelocity wheels;
//   esp_err_t err = drivetrain_kinematics_body_to_wheel_velocities(config, body, wheels);
esp_err_t drivetrain_kinematics_body_to_wheel_velocities(
    const DrivetrainVelocityKinematicsConfig &config,
    const DrivetrainBodyVelocity &body,
    DrivetrainWheelVelocity &wheels_out
);
