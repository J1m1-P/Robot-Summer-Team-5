/* Defines the visual-error -> angular-velocity conversion tuning. */
#include "config/turn_translation_config.h"

// ADJUST: raise if the robot turns too slowly to track a moving/off-center
// target; lower if it overshoots or oscillates. Tune once the drivetrain
// side actually consumes this value.
const float TURN_OMEGA_GAIN_RAD_S = 2.0f;

// ADJUST: bounds what the arm board will ever ask for; the drivetrain's own
// velocity limits may clamp further regardless.
const float TURN_OMEGA_MAX_RAD_S = 3.0f;
