/* Exposes the single hardware, geometry, controller, and safety configuration. */
#pragma once

#include "control/drivetrain/drivetrain.h"

#ifdef __cplusplus
extern "C" {
#endif

// Complete configuration used by production and drivetrain test firmware.
extern const DrivetrainConfig DRIVETRAIN_CONFIG;

#ifdef __cplusplus
}
#endif
