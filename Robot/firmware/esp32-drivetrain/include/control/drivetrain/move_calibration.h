/* Declares immutable calibration data shared by final movement APIs. */
#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Wheel order is fixed to the drivetrain/kinematics order: FL, FR, BL, BR. */
enum { MOVE_CALIBRATION_WHEEL_COUNT = 4 };

typedef enum {
    MOVE_CALIBRATION_POS_X = 0,
    MOVE_CALIBRATION_NEG_X,
    MOVE_CALIBRATION_POS_Y,
    MOVE_CALIBRATION_NEG_Y,
    MOVE_CALIBRATION_DIRECTION_MAX,
} MoveCalibrationDirection;

/* The single source of truth for the measured static-calibration factors.
 * `enabled` remains false until the final advanced-movement integration and
 * its after-calibration validation are complete. */
typedef struct {
    bool enabled;

    /* Representative speed used to collect these single-speed factors. */
    float linear_speed_mps;

    /* Direction-specific longitudinal factors, selected from the sign and
     * dominant component of the requested body translation. */
    float f_lon[MOVE_CALIBRATION_DIRECTION_MAX];

    /* Direction-specific wheel factors. Dimensions are direction then wheel
     * in FL, FR, BL, BR order. */
    float f_lat[MOVE_CALIBRATION_DIRECTION_MAX][MOVE_CALIBRATION_WHEEL_COUNT];

    /* Pure target-angle rotation factor. It is intentionally not applied to
     * instantaneous advanced-command omega. */
    float f_ang;
} MoveCalibrationConfig;

#ifdef __cplusplus
}
#endif
