#include "config/wheel_velocity_pi_config.h"

namespace {

// kKff/kKffOffset: refit from a LOADED closed-loop steady-state
// characterization (wheel on the ground, real chassis weight, ground
// contact friction) across M1-M4 and targets 0.2-0.5 m/s -- see
// misc/pi_verify_range_0.2-0.5_all_motors_ff1.3_kp0.4_ki0.2.csv. Replaces
// the original unloaded fit (kff=0.786, kff_offset=-0.0222), which
// undershot badly once actually loaded -- note the offset even changed
// sign, since a loaded wheel needs real duty to overcome static friction
// that an unloaded one barely needed at all.
//
// Per-motor fits from that data were fairly tight (kff 1.08-1.19,
// offset 0.057-0.086), so one shared value across all 4 motors is a
// reasonable compromise given WheelVelocityPiConfig is shared, not
// per-wheel; these are the average of the 4 per-motor fits.
//
// kp/ki verified adequate (fast rise, small overshoot, no saturation) for
// 0.3-0.5 m/s across all 4 motors -- see the same capture file. Below
// that, expect real undershoot from unmodeled static-friction
// nonlinearity, and M4 specifically is slow to break away from rest at
// low target (mechanical, not a gain problem -- its steady-state fit
// matches the other three).
constexpr float kKff = 1.14f;
constexpr float kKffOffset = 0.069f;
constexpr float kKp = 0.4f;
constexpr float kKi = 0.2f;

// Duty clamp. Independent of DRIVETRAIN_CONFIG.max_duty
// (config/drivetrain_config.c) -- despite both starting at the same 0.4
// value, they are two separate constants with no link between them; the
// [env:tuning] build doesn't even compile drivetrain_config.c. Edit this
// one for anything PI/tuning-related.
constexpr float kMaxDuty = 0.8f;

// TODO: tune once the boost converter/H-bridge is confirmed healthy
// again. A commanded turn at turn_speed_rad_s=1.0 (all 4 motors
// simultaneously) is suspected of frying the boost converter -- without
// this, feedforward alone can jump duty from 0 toward its full computed
// value in a single control cycle the instant a target changes, and 4
// wheels doing that at the same instant (an in-place turn starts all 4
// together) is a simultaneous current inrush that was never
// characterized (every prior duty-vs-speed capture used one wheel at a
// time). 2.0 duty/s means reaching a typical ~0.5 operating duty takes
// ~250ms instead of effectively instantaneous -- a real but
// conservative starting cap, not a measured value. Tighten further if
// the hardware issue turns out to be slew-related; loosen once actual
// current draw is measured and known to be within the boost converter's
// rating.
constexpr float kDutySlewPerS = 2.0f;

}  // namespace

const WheelVelocityPiConfig WHEEL_VELOCITY_PI_CONFIG = {
    kKff,
    kKffOffset,
    kKp,
    kKi,
    -kMaxDuty,
    kMaxDuty,
    -kMaxDuty,
    kMaxDuty,
    kDutySlewPerS,
};
