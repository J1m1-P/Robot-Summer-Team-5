/* Implements the shared endpoint-settle pulse/pause hold controller for
 * angular residual error. See rotational_settle.h for the deadband
 * derivation. */
#include "control/drivetrain/rotational_settle.h"

#include <math.h>
#include <string.h>

#include <robot_common/math_utils.h>

/* Derived, not guessed: 0.05 m/s characterized linear wheel floor
 * (drivetrain_config.c) divided by arm = chassis_half_length_m *
 * sin(wheel_angle_rad) + chassis_half_width_m * cos(wheel_angle_rad) using
 * DRIVETRAIN_CONFIG.x_drive_kinematics's geometry (0.100, 0.1365, 30 deg)
 * -- see rotational_settle.h for the full derivation and why the wheel
 * radius cancels out. Recompute this if that geometry or the characterized
 * wheel floor ever changes. */
static const float kAngularDeadbandRadS = 0.297243f;

/* Everything below this line is a reasoned starting point built around the
 * derived floor above, not itself measured on hardware -- same status as
 * endpoint_settle.c's own kHoldMinSpeed/kHoldMaxSpeed. Verify/adjust after
 * watching real rotation-settle behavior. */
static const float kHoldGain = 2.0f;
static const float kHoldMinRadS = kAngularDeadbandRadS * 1.5f;  // margin to reliably clear the floor
static const float kHoldMaxRadS = 0.6f;   // well under MoveRConfig's max_omega_rad_s ceiling (1.0 rad/s)
static const float kPulseDurationS = 0.08f;
static const float kPulsePauseS = 0.10f;

void rotational_settle_reset(RotationalSettleState *state) {
    memset(state, 0, sizeof(*state));
}

float rotational_settle_deadband_rad_s(void) {
    return kAngularDeadbandRadS;
}

float rotational_settle_update(RotationalSettleState *state,
                               float error_magnitude_rad,
                               float tolerance_rad,
                               float dt_s) {
    if (error_magnitude_rad <= tolerance_rad) {
        return 0.0f;
    }
    if (state->pulse_remaining_s <= 0.0f && state->pause_remaining_s <= 0.0f) {
        state->pulse_remaining_s = kPulseDurationS;
    }
    if (state->pulse_remaining_s > 0.0f) {
        const float hold_omega = clamp(error_magnitude_rad * kHoldGain,
                                       kHoldMinRadS, kHoldMaxRadS);
        state->pulse_remaining_s = fmaxf(0.0f, state->pulse_remaining_s - dt_s);
        if (state->pulse_remaining_s <= 0.0f) {
            state->pause_remaining_s = kPulsePauseS;
        }
        return hold_omega;
    }
    state->pause_remaining_s = fmaxf(0.0f, state->pause_remaining_s - dt_s);
    return 0.0f;
}
