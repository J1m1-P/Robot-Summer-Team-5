#pragma once

#include "esp_err.h"

// Per-wheel velocity PI controller: closes the loop between the velocity
// Jacobian's target wheel speed (control/drivetrain_velocity_kinematics.h,
// converted from rad/s to m/s via r*omega) and the encoder's measured
// speed (drivers/encoder_driver.h's encoder_driver_get_velocity_mps()),
// and outputs a duty cycle for motor_driver_set_duty(). One instance per
// wheel -- construct 4 of these for a full Drivetrain, same way there are
// 4 MotorDriver/EncoderDriver instances.
//
// PI only, no derivative term -- deliberate choice, not a placeholder.
//
// Also carries a feedforward term (kff). This is still a PI controller,
// not a PID: feedforward isn't a feedback term (it doesn't look at
// error), it's an open-loop duty estimate computed straight from the
// target speed and added on top, so the PI only has to correct the
// residual (friction, battery sag, wheel-to-wheel mismatch) instead of
// climbing Ki from zero on every target change.
//
// Units are m/s throughout (not rad/s or rpm) to match
// encoder_driver_get_velocity_mps() directly -- see the unit-consistency
// note from when the velocity kinematics model was compared to the
// encoder feedback path.

// Tuning gains and output limits for one wheel's controller. Feedforward
// + PI form:
//   u(t) = kff*target(t) + sign(target(t))*kff_offset + kp*e(t) + ki*integral(e(t)dt)
// where e(t) is (target speed - measured speed) in m/s and u(t) is a duty
// cycle. The integral accumulator is held at 0 for as long as
// target_mps == 0 (see wheel_velocity_pi.cpp) -- "stop" shouldn't inherit
// however much integral built up while cruising at a prior speed. Also at
// target_mps == 0, output is clamped to pure active braking -- duty may
// only oppose the wheel's current direction of motion, never push it
// past zero into the opposite direction.
struct WheelVelocityPiConfig {
    // Feedforward slope: duty per m/s of target speed, i.e. an
    // approximation of this motor's duty-to-speed slope. Characterize by
    // driving open-loop (kp=ki=0) at a few fixed duties on the tuning
    // build, recording steady-state measured_mps per duty, and fitting
    // duty = kff*speed + kff_offset -- see src/tuning_main.cpp. kff is
    // that fit's slope.
    float kff = 0.0f;

    // Feedforward offset: the fit's intercept -- duty needed beyond the
    // pure kff*target term, e.g. to overcome static friction/deadband.
    // Applied with target's sign (and not at all when target_mps == 0,
    // so commanding a stop never adds a nonzero duty) since the
    // regression this comes from is normally only characterized for one
    // direction (the tuning harness's step test never commands negative
    // targets) -- mirroring the sign is an assumption of symmetric
    // friction, not something actually measured in the opposite
    // direction. Store it exactly as the fit's intercept (can be
    // negative).
    float kff_offset = 0.0f;

    float kp = 0.0f; // proportional gain -- scales the instantaneous error e(t)
    float ki = 0.0f; // integral gain -- scales the accumulated error over time

    // Duty cycle clamp on u(t) -- matches motor_driver's [-max_duty, max_duty] range.
    float output_min = -1.0f;
    float output_max = 1.0f;

    // Anti-windup clamp on the accumulated integral term (independent of
    // output_min/max so windup can be limited more tightly than the duty
    // range itself, if needed).
    float integral_min = -1.0f;
    float integral_max = 1.0f;

    // Maximum allowed |change in duty| per second, applied to the final
    // output (after the output_min/output_max clamp) regardless of
    // direction -- caps how fast commanded duty can ramp up OR down.
    // Without this, feedforward alone can jump duty from 0 to most of its
    // final value in a single control cycle the instant a target changes
    // (kff*target has no ramp of its own), and with 4 wheels doing that
    // in the same instant -- e.g. every wheel starting an in-place turn
    // together -- that's a simultaneous current inrush across all 4
    // motors that was never characterized or budgeted for (every prior
    // duty-vs-speed test used one wheel at a time). Default here
    // (WHEEL_VELOCITY_PI_CONFIG) is a real, finite value for exactly that
    // reason; set to a very large number (or omit) for "effectively
    // unlimited," which is what every test and prior capture implicitly
    // assumed before this field existed.
    float duty_slew_per_s = 1.0e6f;
};

// Per-wheel controller state, carried across successive
// wheel_velocity_pi_update() calls for the same wheel.
struct WheelVelocityPi {
    float integral = 0.0f;  // running accumulator of e(t)*dt, i.e. integral(e(t)dt)
    float last_duty = 0.0f; // previous call's output, for slew-rate limiting
};

// Advances the controller by one control-loop tick of duration dt_s and
// writes the resulting duty cycle (already clamped to
// [output_min, output_max]) into duty_out.
esp_err_t wheel_velocity_pi_update(
    WheelVelocityPi &pi,
    const WheelVelocityPiConfig &config,
    float target_mps,
    float measured_mps,
    float dt_s,
    float &duty_out
);

void wheel_velocity_pi_reset(WheelVelocityPi &pi);
