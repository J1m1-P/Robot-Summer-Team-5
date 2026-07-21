// Test suite for control/drivetrain/wheel_velocity_controller.h, focused on the integral
// accumulator's behavior -- in particular that a "stop" command
// (target_mps == 0) doesn't inherit integral windup from whatever was
// accumulated while cruising at a prior speed.
//
// Run with: pio test -e native

#include <unity.h>

#include "control/drivetrain/wheel_velocity_controller.h"

namespace {

// Keeps test expressions concise while exercising the pointer-based C API.
esp_err_t wheel_velocity_controller_update(
    WheelVelocityController &pi,
    const WheelVelocityControllerConfig &config,
    float target_mps,
    float measured_mps,
    float dt_s,
    float &duty_out
) {
    return ::wheel_velocity_controller_update(
        &pi, &config, target_mps, measured_mps, dt_s, &duty_out);
}

WheelVelocityControllerConfig make_config() {
    WheelVelocityControllerConfig cfg = {};
    cfg.kff = 0.0f;
    cfg.kff_offset = 0.0f;
    cfg.kp = 0.5f;
    cfg.ki = 1.0f; // large on purpose, to make windup obvious if it weren't handled
    cfg.output_min = -1.0f;
    cfg.output_max = 1.0f;
    cfg.integral_min = -10.0f;
    cfg.integral_max = 10.0f;
    cfg.duty_slew_per_s = 1.0e6f;
    return cfg;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_integral_accumulates_normally_while_target_nonzero() {
    const WheelVelocityControllerConfig cfg = make_config();
    WheelVelocityController pi = {};
    float duty = 0.0f;

    // Cruise with a sustained tracking error (measured never quite
    // catches target) -- integral should grow, cycle after cycle.
    wheel_velocity_controller_update(pi, cfg, 1.0f, 0.5f, 0.1f, duty);
    const float integral_after_one = pi.integral;
    TEST_ASSERT_TRUE(integral_after_one > 0.0f);

    wheel_velocity_controller_update(pi, cfg, 1.0f, 0.5f, 0.1f, duty);
    TEST_ASSERT_TRUE(pi.integral > integral_after_one);
}

void test_integral_resets_immediately_when_target_becomes_zero() {
    const WheelVelocityControllerConfig cfg = make_config();
    WheelVelocityController pi = {};
    float duty = 0.0f;

    // Build up a large integral while cruising, as if sustaining speed
    // against friction/load for a while.
    for (int i = 0; i < 20; i++) {
        wheel_velocity_controller_update(pi, cfg, 1.0f, 0.9f, 0.1f, duty);
    }
    TEST_ASSERT_TRUE(pi.integral > 0.0f);

    // Command a stop. Even though the wheel is still coasting at 0.9 m/s
    // (measured hasn't caught up to the new target yet), the integral
    // must not inherit the cruise-phase accumulation.
    wheel_velocity_controller_update(pi, cfg, 0.0f, 0.9f, 0.1f, duty);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pi.integral);
}

void test_integral_stays_zero_while_target_remains_zero_and_wheel_still_coasting() {
    const WheelVelocityControllerConfig cfg = make_config();
    WheelVelocityController pi = {};
    float duty = 0.0f;

    // Simulate a coast-down: target is 0 throughout, measured speed
    // decays toward 0 over several cycles. The integral should stay
    // pinned at 0 the whole time, not slowly accumulate from the
    // nonzero error during the coast-down.
    float measured = 0.9f;
    for (int i = 0; i < 10; i++) {
        wheel_velocity_controller_update(pi, cfg, 0.0f, measured, 0.1f, duty);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, pi.integral);
        measured *= 0.8f; // decaying toward 0
    }
}

void test_stop_command_coasts_to_zero_duty() {
    const WheelVelocityControllerConfig cfg = make_config();
    WheelVelocityController pi = {};
    float duty = 0.0f;

    // Build up integral while cruising.
    for (int i = 0; i < 20; i++) {
        wheel_velocity_controller_update(pi, cfg, 1.0f, 0.9f, 0.1f, duty);
    }

    // Now stop: error = 0 - 0.9 = -0.9, which would drive a nonzero
    // (reverse-polarity, actively braking) duty if it weren't overridden --
    // duty must be exactly 0 (coast) instead, regardless of kp/ki/kff.
    wheel_velocity_controller_update(pi, cfg, 0.0f, 0.9f, 0.1f, duty);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
}

void test_integral_resumes_accumulating_once_target_goes_nonzero_again() {
    const WheelVelocityControllerConfig cfg = make_config();
    WheelVelocityController pi = {};
    float duty = 0.0f;

    wheel_velocity_controller_update(pi, cfg, 1.0f, 0.5f, 0.1f, duty); // accumulate
    wheel_velocity_controller_update(pi, cfg, 0.0f, 0.5f, 0.1f, duty); // stop -> reset to 0
    TEST_ASSERT_EQUAL_FLOAT(0.0f, pi.integral);

    wheel_velocity_controller_update(pi, cfg, 1.0f, 0.5f, 0.1f, duty); // moving again
    TEST_ASSERT_TRUE(pi.integral > 0.0f);
}

void test_stop_never_pushes_forward_even_with_misconfigured_positive_output() {
    WheelVelocityControllerConfig cfg = make_config();
    cfg.kp = -0.5f; // deliberately wrong-signed, to prove the override is a
                     // real enforced invariant and not just an incidental
                     // consequence of kp being positive elsewhere in the file
    WheelVelocityController pi = {};
    float duty = 0.0f;

    // Without the target==0 override, kp*error = -0.5*(0-0.9) = +0.45 --
    // exactly the "push forward while supposed to be stopping" case the
    // override exists to prevent, from a wheel still moving forward at
    // 0.9 m/s. Duty must be exactly 0 (coast), not merely non-positive.
    wheel_velocity_controller_update(pi, cfg, 0.0f, 0.9f, 0.1f, duty);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
}

void test_stop_never_pushes_further_backward_when_wheel_already_reversed() {
    WheelVelocityControllerConfig cfg = make_config();
    cfg.kp = -0.5f; // same defensive misconfiguration, mirrored for the
                     // wheel-already-moving-backward case
    WheelVelocityController pi = {};
    float duty = 0.0f;

    // measured_mps = -0.9 (wheel already spinning backward), target = 0.
    // Without the override, kp*error = -0.5*(0-(-0.9)) = -0.45 -- would
    // drive it further backward instead of coasting it toward 0.
    wheel_velocity_controller_update(pi, cfg, 0.0f, -0.9f, 0.1f, duty);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
}

void test_stop_at_exactly_zero_measured_produces_zero_duty() {
    const WheelVelocityControllerConfig cfg = make_config();
    WheelVelocityController pi = {};
    float duty = 0.0f;

    wheel_velocity_controller_update(pi, cfg, 0.0f, 0.0f, 0.1f, duty);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, duty);
}

void test_slew_limit_caps_first_cycle_jump_from_rest() {
    WheelVelocityControllerConfig cfg = make_config();
    cfg.kff = 2.0f;         // large feedforward so the raw commanded jump is big...
    cfg.duty_slew_per_s = 1.0f; // ...but slew allows only 1.0 duty/s
    WheelVelocityController pi = {};
    float duty = 0.0f;

    // Raw output would be kff*target = 2.0*1.0 = 2.0 (clamped to
    // output_max=1.0) on the very first call -- an instant full-scale
    // jump from rest. With duty_slew_per_s=1.0 and dt=0.1s, the allowed
    // change this cycle is only 0.1.
    wheel_velocity_controller_update(pi, cfg, 1.0f, 0.0f, 0.1f, duty);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, duty);
}

void test_slew_limit_ramps_toward_target_over_multiple_cycles() {
    WheelVelocityControllerConfig cfg = make_config();
    cfg.kff = 2.0f;
    cfg.kp = 0.0f; // isolate the slew behavior from proportional response
    cfg.ki = 0.0f;
    cfg.duty_slew_per_s = 1.0f;
    WheelVelocityController pi = {};
    float duty = 0.0f;

    // Raw commanded output is always 1.0 (clamped). Each 0.1s cycle
    // should move duty by exactly 0.1 toward it until it arrives.
    for (int i = 1; i <= 9; i++) {
        wheel_velocity_controller_update(pi, cfg, 1.0f, 0.0f, 0.1f, duty);
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.1f * i, duty);
    }
    // By cycle 10 it should have arrived at (and stay clamped to) 1.0.
    wheel_velocity_controller_update(pi, cfg, 1.0f, 0.0f, 0.1f, duty);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, duty);
}

void test_safe_stop_overrides_downward_slew_limit() {
    WheelVelocityControllerConfig cfg = make_config();
    cfg.kff = 1.0f;
    cfg.kp = 0.0f;
    cfg.ki = 0.0f;
    cfg.duty_slew_per_s = 1.0f;
    WheelVelocityController pi = {};
    float duty = 0.0f;

    // Get to steady duty=1.0 first (kff*target = 1.0*1.0, within one
    // slew-limited step since 1.0 duty/s * 1.0s = 1.0 allowed this cycle).
    wheel_velocity_controller_update(pi, cfg, 1.0f, 0.0f, 1.0f, duty);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, duty);

    // A zero target must stop driving forward immediately even though the
    // ordinary symmetric slew rule would otherwise return positive duty.
    wheel_velocity_controller_update(pi, cfg, 0.0f, 1.0f, 0.1f, duty);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, duty);
}

void test_default_slew_is_effectively_unlimited() {
    // make_config() deliberately uses a very high slew rate so ordinary
    // controller tests are not limited by output ramping.
    const WheelVelocityControllerConfig cfg = make_config();
    WheelVelocityController pi = {};
    float duty = 0.0f;

    wheel_velocity_controller_update(pi, cfg, 1.0f, 0.0f, 0.001f, duty); // very small dt
    // kp*error = 0.5*1.0 = 0.5, reached immediately despite the tiny dt.
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.5f, duty);
}

void test_rejects_invalid_controller_limits() {
    WheelVelocityControllerConfig cfg = make_config();
    cfg.output_min = 1.0f;
    cfg.output_max = -1.0f;
    WheelVelocityController pi = {};
    float duty = 0.0f;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        wheel_velocity_controller_update(pi, cfg, 1.0f, 0.0f, 0.1f, duty));
}

void test_rejects_non_finite_controller_state() {
    const WheelVelocityControllerConfig cfg = make_config();
    WheelVelocityController pi = {};
    pi.integral = NAN;
    float duty = 0.0f;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        wheel_velocity_controller_update(pi, cfg, 1.0f, 0.0f, 0.1f, duty));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_integral_accumulates_normally_while_target_nonzero);
    RUN_TEST(test_integral_resets_immediately_when_target_becomes_zero);
    RUN_TEST(test_integral_stays_zero_while_target_remains_zero_and_wheel_still_coasting);
    RUN_TEST(test_stop_command_coasts_to_zero_duty);
    RUN_TEST(test_integral_resumes_accumulating_once_target_goes_nonzero_again);

    RUN_TEST(test_stop_never_pushes_forward_even_with_misconfigured_positive_output);
    RUN_TEST(test_stop_never_pushes_further_backward_when_wheel_already_reversed);
    RUN_TEST(test_stop_at_exactly_zero_measured_produces_zero_duty);

    RUN_TEST(test_slew_limit_caps_first_cycle_jump_from_rest);
    RUN_TEST(test_slew_limit_ramps_toward_target_over_multiple_cycles);
    RUN_TEST(test_safe_stop_overrides_downward_slew_limit);
    RUN_TEST(test_default_slew_is_effectively_unlimited);
    RUN_TEST(test_rejects_invalid_controller_limits);
    RUN_TEST(test_rejects_non_finite_controller_state);

    return UNITY_END();
}
