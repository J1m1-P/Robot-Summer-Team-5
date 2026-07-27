/* Exposes the closed-loop MOTION primitives' production tuning. */
#pragma once

#include "control/drivetrain/move_c.h"
#include "control/drivetrain/move_l.h"
#include "control/drivetrain/move_p.h"
#include "control/drivetrain/move_r.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Values carried over from src/harnesses/calibration_main.cpp's tuned
 * defaults -- the only known-good values for these primitives in the repo.
 * Re-tune here after a new completed calibration trial set. */
extern const MoveLConfig MOVE_L_CONFIG;
extern const MovePConfig MOVE_P_CONFIG;
extern const MoveRConfig MOVE_R_CONFIG;
extern const MoveCConfig MOVE_C_CONFIG;

#ifdef __cplusplus
}
#endif
