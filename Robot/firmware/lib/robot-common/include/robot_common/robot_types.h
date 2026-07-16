/*
 * Defines general-purpose robot state and command data shared between modules.
 * Units are included in field names to make conversions explicit.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Describes a requested planar robot pose.
typedef struct {
    float target_x_m;
    float target_y_m;
    float target_theta_rad;
} RobotPoseTarget;

// Describes a measured planar robot pose.
typedef struct {
    float x_m;
    float y_m;
    float theta_rad;
} RobotPose2D;

// Describes planar linear and angular velocity.
typedef struct {
    float vx_mps;
    float vy_mps;
    float omega_radps;
} RobotVelocity2D;

// Combines a timestamped pose and velocity estimate with a validity flag.
typedef struct {
    RobotPose2D pose;
    RobotVelocity2D velocity;
    uint32_t timestamp_ms;
    bool valid;
} RobotOdometry;

// Describes normalized body-axis translation and turning duty commands.
typedef struct {
    float x_duty;
    float y_duty;
    float turn_duty;
} RobotBodyDutyCommand;

#ifdef __cplusplus
}
#endif
