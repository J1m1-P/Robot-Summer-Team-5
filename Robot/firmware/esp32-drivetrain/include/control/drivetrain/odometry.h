/* Declares cumulative drivetrain pose integration and odometry fault state. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Represents one body-frame motion delta received from the sensing system.
typedef struct {
    float forward_mm;
    float lateral_mm;
    float heading_delta_rad;
} DrivetrainOdometryDelta;

// Represents the cumulative world-frame drivetrain pose.
typedef struct {
    float x_mm;
    float y_mm;
    float heading_rad;
} DrivetrainPose;

// Records whether the most recent odometry cycle was usable.
typedef enum {
    DRIVETRAIN_ODOMETRY_FAULT_NONE = 0,
    DRIVETRAIN_ODOMETRY_FAULT_INVALID_CYCLE,
} DrivetrainOdometryFault;

// Stores cumulative pose and the current placeholder odometry fault policy.
typedef struct {
    DrivetrainPose pose;
    bool fault_latched;
    DrivetrainOdometryFault last_fault;
} DrivetrainOdometry;

// Integrates one valid body delta or records an invalid-cycle fault.
esp_err_t drivetrain_odometry_update(
    DrivetrainOdometry *odometry,
    const DrivetrainOdometryDelta *delta,
    bool valid
);

// Clears accumulated pose and all odometry fault state.
void drivetrain_odometry_reset(DrivetrainOdometry *odometry);

#ifdef __cplusplus
}
#endif
