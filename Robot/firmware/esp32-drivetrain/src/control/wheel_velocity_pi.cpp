#include "control/wheel_velocity_pi.h"

#include <cmath>

namespace {

float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

}  // namespace

esp_err_t wheel_velocity_pi_update(
    WheelVelocityPi &pi,
    const WheelVelocityPiConfig &config,
    float target_mps,
    float measured_mps,
    float dt_s,
    float &duty_out
) {
    if (!std::isfinite(target_mps) || !std::isfinite(measured_mps) || !std::isfinite(dt_s) || dt_s <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    // Feedforward + PI: u(t) = Kff*target(t) + Kp*e(t) + Ki*integral(e(t)dt)
    //   target(t) -- the setpoint itself, feeding the open-loop term
    //   e(t)      -- error: setpoint minus measured process variable
    //   Kff/Kp/Ki -- config.kff/kp/ki, the three tuning gains
    //   u(t)      -- controller output, here a motor duty cycle in [-1, 1]

    // e(t) = target_mps - measured_mps: setpoint (desired wheel speed)
    // minus process variable (measured wheel speed), both in m/s.
    const float error = target_mps - measured_mps;

    // Integral term: running sum of e(t)*dt approximates integral(e dt).
    // pi.integral is that accumulator (persists across calls); clamped
    // to [integral_min, integral_max] as anti-windup, so a long-standing
    // error can't accumulate an unbounded correction that overshoots once
    // the error finally clears.
    //
    // Special case: target_mps == 0 means "stop", and stopping shouldn't
    // depend on however much integral happened to accumulate while
    // cruising at the previous speed. Without this, commanding a stop
    // from a sustained cruise leaves a large pi.integral that only decays
    // as fast as the (now strongly negative) error pulls it down --
    // during that decay, ki*integral keeps contributing output long after
    // the stop was requested, which can overshoot into reverse or leave
    // the wheel creeping even after it's physically stopped. Pinning
    // integral to 0 for the whole time target_mps == 0 means the only
    // thing braking the wheel is the proportional term (kp*error, still
    // fully active since measured_mps probably isn't 0 yet) -- decisive,
    // no leftover bias once the wheel actually reaches 0.
    if (target_mps == 0.0f) {
        pi.integral = 0.0f;
    } else {
        pi.integral = clamp(pi.integral + error * dt_s, config.integral_min, config.integral_max);
    }

    // u(t) = Kff*target(t) + sign(target)*Kff_offset + Kp*e(t) + Ki*integral_accumulator,
    // clamped to the configured duty range. Feedforward depends only on
    // the setpoint, not on error or accumulated state -- it's not part of
    // the feedback loop, just an open-loop estimate added on top of it.
    // kff_offset is applied with target's sign and omitted entirely at
    // target_mps == 0, so commanding a stop never adds a nonzero duty.
    float feedforward_term = config.kff * target_mps;
    if (target_mps > 0.0f) {
        feedforward_term += config.kff_offset;
    } else if (target_mps < 0.0f) {
        feedforward_term -= config.kff_offset;
    }

    const float proportional_term = config.kp * error;
    const float integral_term = config.ki * pi.integral;

    float output = feedforward_term + proportional_term + integral_term;

    // Stopping (target_mps == 0) must be pure active braking: duty may
    // only oppose whichever direction the wheel is currently moving, and
    // must never push it past zero into the opposite direction. kp*error
    // is already always braking-direction by construction here (error =
    // -measured_mps when target is 0, so it's structurally opposite in
    // sign to measured_mps) -- these clamps make that an explicit,
    // enforced invariant rather than an incidental property that a future
    // change (e.g. a nonzero feedforward term someone adds later) could
    // silently break. They don't prevent physical momentum from carrying
    // the wheel past zero on its own, but they guarantee the controller
    // itself never assists an overshoot -- once measured_mps reaches
    // exactly 0, error is 0 too and duty falls out to 0 naturally, no
    // separate zero-crossing case needed.
    if (target_mps == 0.0f) {
        if (measured_mps > 0.0f) {
            output = fminf(output, 0.0f); // still moving forward -- braking duty only, never positive
        } else if (measured_mps < 0.0f) {
            output = fmaxf(output, 0.0f); // still moving backward -- braking duty only, never negative
        }
    }

    const float clamped_output = clamp(output, config.output_min, config.output_max);

    // Slew-rate limit: cap how far duty_out can move from last call's
    // output in this one dt_s tick, regardless of direction. See the
    // duty_slew_per_s comment in wheel_velocity_pi.h for why this exists
    // (inrush current from an instant feedforward step, especially with
    // multiple wheels stepping at the same instant).
    const float max_delta = config.duty_slew_per_s * dt_s;
    const float delta = clamped_output - pi.last_duty;
    if (delta > max_delta) {
        duty_out = pi.last_duty + max_delta;
    } else if (delta < -max_delta) {
        duty_out = pi.last_duty - max_delta;
    } else {
        duty_out = clamped_output;
    }

    pi.last_duty = duty_out;

    return ESP_OK;
}

void wheel_velocity_pi_reset(WheelVelocityPi &pi) {
    pi = WheelVelocityPi{};
}
