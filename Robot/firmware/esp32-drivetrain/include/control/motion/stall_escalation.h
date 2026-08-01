#pragma once

// Tracks how long a commanded translation has failed to produce measurable
// progress and grows a scale factor to inflate the whole commanded
// vx/vy vector by, so all wheels ramp up together -- preserving the
// commanded direction -- until static friction releases and the drivetrain
// starts moving. Scale decays back to 1.0 as soon as progress resumes.
//
// kStallGraceS/kEscalationPerS/kMaxScale (see stall_escalation.cpp) are
// initial estimates, not measured values -- validate and retune them against
// real drivetrain behavior before relying on this in the field.
typedef struct {
    float stalled_s;
    float scale;
} StallEscalation;

// Advances stall tracking by one control cycle and returns the multiplier to
// apply to the commanded vx/vy via stall_escalation_apply_scale().
// `commanded_speed_mps` is hypot(vx, vy) of that command; `progress_m` is how
// far the robot actually moved (from odometry/pose) over this cycle.
float stall_escalation_update(
    StallEscalation *state, float commanded_speed_mps, float progress_m, float dt_s);

// Scales *vx and *vy in place by `scale`, then rescales both together --
// never just one -- so their ratio (the commanded direction) is unchanged,
// while keeping the result within the given per-axis limits. Centralizing
// this here means both callers get the same direction-preserving clamp
// instead of each clamping vx and vy independently, which can skew direction
// whenever max_vx_mps and max_vy_mps differ.
void stall_escalation_apply_scale(
    float *vx, float *vy, float scale, float max_vx_mps, float max_vy_mps);
