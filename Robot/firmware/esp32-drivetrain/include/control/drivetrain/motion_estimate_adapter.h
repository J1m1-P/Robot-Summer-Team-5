/* Adapts the current encoder odometry into the generic advanced-motion input. */
#pragma once

#include <stdbool.h>

#include "esp_err.h"

#include "control/drivetrain/odometry.h"
#include "control/drivetrain/path_planner.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Converts the present millimetre world pose to the metre world contract used
 * by MoveP and future advanced APIs.  `estimate_valid` is supplied by the
 * active estimator's health policy; replacing encoder odometry with PMW3610
 * fusion changes only this boundary. */
esp_err_t motion_estimate_from_drivetrain_pose(
    const DrivetrainPose *pose,
    bool estimate_valid,
    MotionEstimate *estimate_out);

#ifdef __cplusplus
}
#endif
