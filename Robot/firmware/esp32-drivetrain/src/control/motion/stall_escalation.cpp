#include "control/motion/stall_escalation.h"

#include <cmath>

namespace {
// Below this commanded speed there's nothing worth breaking free from --
// treat it as "not really commanding translation" and don't escalate.
constexpr float kMinCommandedSpeedMps = 0.05f;
// Progress below this over the elapsed dt counts as "not moving" for a wheel
// meant to be rolling at kMinCommandedSpeedMps or faster.
constexpr float kStalledSpeedMps = 0.07f;
// Grace period before escalating, so the normal startup ramp isn't mistaken
// for a stall. Initial estimate -- not yet validated against real odometry
// update cadence, which could be noisier/slower than the 200 Hz control loop
// and cause spurious single-cycle "no progress" reads.
constexpr float kStallGraceS = 0.5f;
// How fast the scale grows once past the grace period, and its ceiling.
// The overall move timeout each caller already enforces is the backstop if
// the wheel never breaks free even at max scale. Also unvalidated estimates.
constexpr float kEscalationPerS = 4.0f;
constexpr float kMaxScale = 3.0f;
}  // namespace

float stall_escalation_update(
    StallEscalation *state, float commanded_speed_mps, float progress_m, float dt_s) {
    if (state == nullptr || dt_s <= 0.0f) return 1.0f;

    if (commanded_speed_mps < kMinCommandedSpeedMps ||
        progress_m >= kStalledSpeedMps * dt_s) {
        state->stalled_s = 0.0f;
        state->scale = 1.0f;
        return 1.0f;
    }

    state->stalled_s += dt_s;
    if (state->stalled_s <= kStallGraceS) {
        state->scale = 1.0f;
        return 1.0f;
    }

    state->scale = 1.0f + kEscalationPerS * (state->stalled_s - kStallGraceS);
    if (state->scale > kMaxScale) state->scale = kMaxScale;
    return state->scale;
}

void stall_escalation_apply_scale(
    float *vx, float *vy, float scale, float max_vx_mps, float max_vy_mps) {
    if (vx == nullptr || vy == nullptr) return;

    const float scaled_vx = *vx * scale;
    const float scaled_vy = *vy * scale;

    // A single shared overshoot factor (not two independent per-axis clamps)
    // keeps vx:vy -- and therefore the commanded direction -- unchanged even
    // when max_vx_mps and max_vy_mps differ.
    float overshoot = 1.0f;
    if (max_vx_mps > 0.0f) {
        overshoot = std::fmax(overshoot, std::fabs(scaled_vx) / max_vx_mps);
    }
    if (max_vy_mps > 0.0f) {
        overshoot = std::fmax(overshoot, std::fabs(scaled_vy) / max_vy_mps);
    }

    *vx = scaled_vx / overshoot;
    *vy = scaled_vy / overshoot;
}
