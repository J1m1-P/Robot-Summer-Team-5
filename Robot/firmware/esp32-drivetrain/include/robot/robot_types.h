#pragma once

#include <stdint.h>
#include <stdbool.h>

// Common data structures that the systems would be using to communicate

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float target_x_m;
    float target_y_m;
    float target_theta_rad;
} RobotPoseTarget;

typedef struct {
    float x_m;
    float y_m;
    float theta_rad;
} RobotPose2D;

typedef struct {
    float vx_mps;
    float vy_mps;
    float omega_radps;
} RobotVelocity2D;

typedef struct {
    RobotPose2D pose;
    RobotVelocity2D velocity;

    uint32_t timestamp_ms;
    bool valid;
} RobotOdometry;

typedef struct {
    float x_duty;
    float y_duty;
    float turn_duty;
} RobotBodyDutyCommand;

#ifdef __cplusplus
}
#endif