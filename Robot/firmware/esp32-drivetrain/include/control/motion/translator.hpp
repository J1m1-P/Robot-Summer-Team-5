#pragma once

#include "esp_err.h"
#include "control/drivetrain/drivetrain.h"
#include "control/odometry/pose_service.h"

struct PrecisionMoveContext {
    Drivetrain *drivetrain;
    PoseService *pose_service;
};

struct PrecisionMoveTarget {
    float dx_body_m;
    float dy_body_m;
    float delta_heading_rad;
    DrivetrainBodyVelocity body_velocity;  // cruise/feed-forward velocity
};

// Runs a body-relative translation and heading change at 200 Hz. The current
// pose is captured when the action starts; all targets are relative to it.
// The call blocks until settled or timeout, and always stops the drivetrain.
esp_err_t precision_move(const PrecisionMoveContext *context,
                         const PrecisionMoveTarget *target,
                         float timeout_s);
