/* Exposes the complete drivetrain hardware and kinematics configuration. */
#pragma once

#include "control/drivetrain.h"

#ifdef __cplusplus
extern "C" {
#endif

// Complete hardware, wheel geometry, and duty-limit configuration.
extern const DrivetrainConfig DRIVETRAIN_CONFIG;

#ifdef __cplusplus
}
#endif
