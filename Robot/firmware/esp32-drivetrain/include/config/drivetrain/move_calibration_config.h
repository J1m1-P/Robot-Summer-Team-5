/* Exposes the one authoritative set of measured movement-calibration values. */
#pragma once

#include "control/drivetrain/move_calibration.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Update only this definition after a new completed calibration trial set. */
extern const MoveCalibrationConfig MOVE_CALIBRATION_CONFIG;

#ifdef __cplusplus
}
#endif
