/* Exposes the gain and limit used to convert the Pi's visual TURN error into
 * an angular velocity command for the drivetrain. */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

// Commanded angular velocity (rad/s) per unit of normalized visual error (-1..1).
extern const float TURN_OMEGA_GAIN_RAD_S;

// Clamps the commanded angular velocity so a full-scale error can't over-command.
extern const float TURN_OMEGA_MAX_RAD_S;

#ifdef __cplusplus
}
#endif
