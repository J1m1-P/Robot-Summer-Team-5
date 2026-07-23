/* Implements the shared endpoint-settle pulse/pause hold controller. */
#include "control/drivetrain/endpoint_settle.h"

#include <math.h>
#include <string.h>

#include <robot_common/math_utils.h>

static const float kHoldGain = 2.0f;
/* The characterized wheel loop has an effective floor near 0.05 m/s. Use a
 * noticeably larger correction so the projected wheel targets clear that
 * deadband during endpoint adjustment -- bumped up (was 0.08-0.12, 50ms
 * pulse) now that duty_slew_per_s no longer caps the wheel loop's response
 * within a pulse (see drivetrain_config.c): the pulse can actually reach a
 * higher target within its duration instead of being slew-limited to a
 * fraction of it, so there's no longer a reason to keep the hold speed and
 * pulse duration this conservative. */
static const float kHoldMinSpeed = 0.10f;
static const float kHoldMaxSpeed = 0.20f;
static const float kPulseDurationS = 0.08f;
static const float kPulsePauseS = 0.10f;

void endpoint_settle_reset(EndpointSettleState *state) {
    memset(state, 0, sizeof(*state));
}

float endpoint_settle_update(EndpointSettleState *state,
                             float error_magnitude_m,
                             float tolerance_m,
                             float dt_s) {
    if (error_magnitude_m <= tolerance_m) {
        return 0.0f;
    }
    if (state->pulse_remaining_s <= 0.0f && state->pause_remaining_s <= 0.0f) {
        state->pulse_remaining_s = kPulseDurationS;
    }
    if (state->pulse_remaining_s > 0.0f) {
        const float hold_speed = clamp(error_magnitude_m * kHoldGain,
                                       kHoldMinSpeed, kHoldMaxSpeed);
        state->pulse_remaining_s = fmaxf(0.0f, state->pulse_remaining_s - dt_s);
        if (state->pulse_remaining_s <= 0.0f) {
            state->pause_remaining_s = kPulsePauseS;
        }
        return hold_speed;
    }
    state->pause_remaining_s = fmaxf(0.0f, state->pause_remaining_s - dt_s);
    return 0.0f;
}
