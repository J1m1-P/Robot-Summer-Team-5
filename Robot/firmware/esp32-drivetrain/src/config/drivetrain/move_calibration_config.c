/* Defines the measured single-speed movement-calibration values. */
#include "config/drivetrain/move_calibration_config.h"

/*
 * Locked in from the three-trial calibration session at 0.30 m/s and
 * 60 deg/s.  Wheel order is FL, FR, BL, BR.  Keep this file as the sole
 * location for replacing measured factors after a future calibration set.
 *
 * F_lon values:
 *   +x = 0.9225, -x = 0.9030, +y = 1.2808, -y = 1.2709
 * F_ang = 0.93755 (90 deg commanded, approximately 96 deg measured).
 *
 * The opposing-direction averages are deliberately not used here: retaining
 * the measured direction entries lets hardware validation identify backlash
 * or traction asymmetry before we decide to collapse them.
 */
const MoveCalibrationConfig MOVE_CALIBRATION_CONFIG = {
    .enabled = true,
    .linear_speed_mps = 0.30f,

    .f_lon = {
        [MOVE_CALIBRATION_POS_X] = 0.9225f,
        [MOVE_CALIBRATION_NEG_X] = 0.9030f,
        [MOVE_CALIBRATION_POS_Y] = 1.2808f,
        [MOVE_CALIBRATION_NEG_Y] = 1.2709f,
    },

    .f_lat = {
        [MOVE_CALIBRATION_POS_X] = {1.0043f, 0.9957f, 1.0043f, 0.9957f},
        [MOVE_CALIBRATION_NEG_X] = {0.9796f, 1.0204f, 0.9796f, 1.0204f},
        [MOVE_CALIBRATION_POS_Y] = {0.9944f, 0.9944f, 1.0056f, 1.0056f},
        [MOVE_CALIBRATION_NEG_Y] = {1.0261f, 1.0261f, 0.9739f, 0.9739f},
    },

    .f_ang = 0.93755f,
};
