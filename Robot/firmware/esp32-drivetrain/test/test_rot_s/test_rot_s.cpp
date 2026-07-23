/* Tests RotS (open-loop, dead-reckoned in-place rotation) without ESP32 hardware. */
#include <unity.h>

#include <cmath>

#include "control/drivetrain/rot_s.h"

namespace {

constexpr float kTolerance = 1.0e-4f;
constexpr float kPi = 3.14159265358979323846f;

RotSConfig make_config(float angle_tolerance_rad = 0.02f, float max_alpha_rad_s2 = 1.5f) {
    RotSConfig config = {};
    config.speed_profile.max_jerk_mps3 = 10.0f;
    config.angle_tolerance_rad = angle_tolerance_rad;
    config.max_alpha_rad_s2 = max_alpha_rad_s2;
    return config;
}

// Keeps test expressions concise while exercising the pointer-based C API.
esp_err_t rot_s_start(
    RotS &rot,
    const RotSConfig &config,
    float angle_rad,
    float max_omega_rad_s
) {
    return ::rot_s_start(&rot, &config, angle_rad, max_omega_rad_s);
}

esp_err_t rot_s_update(
    RotS &rot,
    float dt_s,
    RotSOutput &output
) {
    return ::rot_s_update(&rot, dt_s, &output);
}

}  // namespace

void setUp() {}
void tearDown() {}

// Confirms a bad config (either angle tolerance or the angular acceleration
// ceiling) is rejected.
void test_rejects_invalid_config() {
    RotS rot = {};

    RotSConfig bad_tolerance = make_config();
    bad_tolerance.angle_tolerance_rad = -1.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, rot_s_start(
        rot, bad_tolerance, kPi / 2.0f, 1.0f));

    RotSConfig bad_alpha = make_config();
    bad_alpha.max_alpha_rad_s2 = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, rot_s_start(
        rot, bad_alpha, kPi / 2.0f, 1.0f));
}

// Confirms start rejects a zero angle and non-positive omega.
void test_start_rejects_invalid_parameters() {
    const RotSConfig config = make_config();
    RotS rot = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, rot_s_start(
        rot, config, 0.0f, 1.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, rot_s_start(
        rot, config, kPi / 2.0f, 0.0f));
}

// Confirms update before start reports invalid state.
void test_update_before_start_is_invalid_state() {
    RotS rot = {};
    RotSOutput output = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, rot_s_update(rot, 0.02f, output));
}

// Confirms a positive (CCW) angle commands positive omega with vx/vy at zero.
void test_positive_angle_commands_positive_omega() {
    const RotSConfig config = make_config();
    RotS rot = {};
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_start(rot, config, kPi / 2.0f, 1.0f));

    RotSOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_update(rot, 0.02f, output));

    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, output.requested_velocity.vx);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, output.requested_velocity.vy);
    TEST_ASSERT_TRUE(output.requested_velocity.omega > 0.0f);
    TEST_ASSERT_EQUAL(static_cast<int>(ROT_S_RUNNING), static_cast<int>(output.status));
}

// Confirms a negative (CW) angle commands negative omega.
void test_negative_angle_commands_negative_omega() {
    const RotSConfig config = make_config();
    RotS rot = {};
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_start(rot, config, -kPi / 2.0f, 1.0f));

    RotSOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_update(rot, 0.02f, output));

    TEST_ASSERT_TRUE(output.requested_velocity.omega < 0.0f);
}

// Confirms remaining angle shrinks by exactly commanded_omega * dt each
// cycle -- RotS is genuinely open-loop (see rot_s.h): there is no heading
// input at all anymore, so "remaining" can only ever come from this
// module's own self-integrated planned_progress_rad, which is a plain
// float accumulation (never any unwrap logic to get wrong, unlike the old
// "subtract against a captured start heading" design this replaced).
void test_remaining_angle_tracks_self_integrated_progress() {
    const RotSConfig config = make_config();
    RotS rot = {};
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_start(rot, config, kPi / 2.0f, 1.0f));

    RotSOutput output_a = {};
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_update(rot, 0.02f, output_a));

    RotSOutput output_b = {};
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_update(rot, 0.02f, output_b));

    TEST_ASSERT_FLOAT_WITHIN(
        kTolerance,
        output_a.remaining_angle_rad - output_a.requested_velocity.omega * 0.02f,
        output_b.remaining_angle_rad);
}

// Confirms a full simulated run reaches completion within the configured
// tolerance. There is no heading to integrate here at all anymore -- RotS's
// own planned_progress_rad does all the tracking -- so this test only needs
// to keep calling rot_s_update() with a fixed dt. Jerk is left effectively
// unconstrained so this test isolates RotS's own
// progress-tracking/stopping-distance/completion logic, mirroring
// test_move_s's equivalent simulated-run test.
void test_simulated_run_reaches_target_angle() {
    RotSConfig config = make_config(0.01f, 2.0f);
    config.speed_profile.max_jerk_mps3 = 1000.0f;
    RotS rot = {};
    const float angle_rad = kPi / 2.0f;
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_start(rot, config, angle_rad, 1.0f));

    constexpr float kDt = 0.01f;
    RotSOutput output = {};
    int iterations = 0;
    constexpr int kMaxIterations = 5000;  // 50s simulated cap
    while (output.status != ROT_S_COMPLETE && iterations < kMaxIterations) {
        TEST_ASSERT_EQUAL(ESP_OK, rot_s_update(rot, kDt, output));
        ++iterations;
    }

    TEST_ASSERT_TRUE(iterations < kMaxIterations);
    TEST_ASSERT_EQUAL(static_cast<int>(ROT_S_COMPLETE), static_cast<int>(output.status));
    TEST_ASSERT_TRUE(fabsf(output.remaining_angle_rad) <= config.angle_tolerance_rad + kTolerance);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.vy);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.omega);
}

// Confirms a full simulated run also converges for a negative (CW) target.
void test_simulated_run_reaches_negative_target_angle() {
    RotSConfig config = make_config(0.01f, 2.0f);
    config.speed_profile.max_jerk_mps3 = 1000.0f;
    RotS rot = {};
    const float angle_rad = -kPi / 3.0f;
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_start(rot, config, angle_rad, 1.0f));

    constexpr float kDt = 0.01f;
    RotSOutput output = {};
    int iterations = 0;
    constexpr int kMaxIterations = 5000;
    while (output.status != ROT_S_COMPLETE && iterations < kMaxIterations) {
        TEST_ASSERT_EQUAL(ESP_OK, rot_s_update(rot, kDt, output));
        ++iterations;
    }

    TEST_ASSERT_TRUE(iterations < kMaxIterations);
    TEST_ASSERT_EQUAL(static_cast<int>(ROT_S_COMPLETE), static_cast<int>(output.status));
    TEST_ASSERT_TRUE(fabsf(output.remaining_angle_rad) <= config.angle_tolerance_rad + kTolerance);
}

// NOTE: the previous revision of this file had
// test_overshoot_past_{positive,negative}_target_reverses_direction here --
// regression tests for a real hardware bug where a caller-supplied heading
// that had overshot past the target caused the old implementation to keep
// commanding the original direction forever instead of reversing to
// correct. Those tests can no longer be written: rot_s_update() no longer
// takes a heading parameter at all (see rot_s.h -- RotS is now genuinely
// open-loop, tracking progress via planned_progress_rad, which only ever
// moves in response to this module's own commanded omega). The bug class
// itself is now structurally unreachable through the public API rather than
// just patched, so there is nothing left to regression-test here. The
// copysignf direction fix remains in rot_s.c as correct, cheap insurance.

// Regression test for a real hardware bug: with irregular (real-world) dt_s
// -- unlike the fixed-dt=0.01 tests above -- commanded omega was observed
// climbing to nearly 3x max_omega_rad_s and never recovering.
// speed_profile_update()'s overshoot-clamp only guarantees its output won't
// cross THIS cycle's target, which is not the same as an absolute ceiling
// when the target itself is changing cycle to cycle (as it does right where
// deceleration begins). Runs with a deliberately irregular mix of tiny and
// large dt_s values (a stand-in for real loop jitter/blocking I/O) and
// asserts the commanded magnitude never exceeds max_omega_rad_s at any
// single call, not just eventually.
void test_commanded_omega_never_exceeds_max_even_with_irregular_dt() {
    RotSConfig config = make_config(0.02f, 1.5f);
    config.speed_profile.max_jerk_mps3 = 1.0f; // the low default that exposed this on hardware
    RotS rot = {};
    const float maxOmega = 1.0f;
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_start(rot, config, kPi / 2.0f, maxOmega));

    const float dtPattern[] = {0.001f, 0.05f, 0.001f, 0.001f, 0.08f, 0.001f, 0.001f, 0.001f, 0.03f, 0.001f};
    RotSOutput output = {};
    for (int i = 0; i < 2000; ++i) {
        const float dt = dtPattern[i % (sizeof(dtPattern) / sizeof(dtPattern[0]))];
        TEST_ASSERT_EQUAL(ESP_OK, rot_s_update(rot, dt, output));
        TEST_ASSERT_TRUE(fabsf(output.requested_velocity.omega) <= maxOmega + kTolerance);
        if (output.status == ROT_S_COMPLETE) break;
    }
}

// The calibration primitive must be a single, one-way maneuver even when a
// low jerk makes its brake ramp long.  This reproduces the configuration that
// previously oscillated on hardware and proves that braking never commands a
// corrective turn in the opposite direction.
void test_low_jerk_rotation_brakes_once_without_reversing() {
    RotSConfig config = make_config(0.02f, 1.5f);
    config.speed_profile.max_jerk_mps3 = 1.0f;
    RotS rot = {};
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_start(rot, config, kPi / 2.0f, kPi / 6.0f));

    RotSOutput output = {};
    bool saw_braking = false;
    int iterations = 0;
    constexpr int kMaxIterations = 5000;
    while (output.status != ROT_S_COMPLETE && iterations < kMaxIterations) {
        TEST_ASSERT_EQUAL(ESP_OK, rot_s_update(rot, 0.01f, output));
        saw_braking = saw_braking || rot.braking;
        TEST_ASSERT_TRUE(output.requested_velocity.omega >= -kTolerance);
        ++iterations;
    }

    TEST_ASSERT_TRUE(saw_braking);
    TEST_ASSERT_TRUE(iterations < kMaxIterations);
    TEST_ASSERT_EQUAL(static_cast<int>(ROT_S_COMPLETE), static_cast<int>(output.status));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_rejects_invalid_config);
    RUN_TEST(test_start_rejects_invalid_parameters);
    RUN_TEST(test_update_before_start_is_invalid_state);
    RUN_TEST(test_positive_angle_commands_positive_omega);
    RUN_TEST(test_negative_angle_commands_negative_omega);
    RUN_TEST(test_remaining_angle_tracks_self_integrated_progress);
    RUN_TEST(test_simulated_run_reaches_target_angle);
    RUN_TEST(test_simulated_run_reaches_negative_target_angle);
    RUN_TEST(test_commanded_omega_never_exceeds_max_even_with_irregular_dt);
    RUN_TEST(test_low_jerk_rotation_brakes_once_without_reversing);
    return UNITY_END();
}
