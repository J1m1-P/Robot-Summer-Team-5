/* Tests the jerk-bounded speed ramp without ESP32 hardware. */
#include <unity.h>

#include <cmath>

#include "control/drivetrain/speed_profile.h"

namespace {

constexpr float kTolerance = 1.0e-4f;

SpeedProfileConfig make_config(float max_jerk_mps3) {
    SpeedProfileConfig config = {};
    config.max_jerk_mps3 = max_jerk_mps3;
    return config;
}

// Keeps test expressions concise while exercising the pointer-based C API.
esp_err_t speed_profile_update(
    SpeedProfile &profile,
    const SpeedProfileConfig &config,
    float target_speed_mps,
    float max_accel_mps2,
    float dt_s,
    float &commanded_speed_mps_out
) {
    return ::speed_profile_update(
        &profile, &config, target_speed_mps, max_accel_mps2, dt_s, &commanded_speed_mps_out);
}

}  // namespace

void setUp() {}
void tearDown() {}

// Confirms reset seeds the ramp at a known speed with zero acceleration.
void test_reset_seeds_initial_speed() {
    SpeedProfile profile = {};
    speed_profile_reset(&profile, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.5f, profile.commanded_speed_mps);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, profile.commanded_accel_mps2);
}

// Confirms an unbounded (high accel/jerk) ramp reaches the target in one
// cycle without overshoot, exercising the exact-landing arithmetic.
void test_reaches_target_in_one_cycle_when_unbounded() {
    SpeedProfile profile = {};
    speed_profile_reset(&profile, 0.0f);
    const SpeedProfileConfig config = make_config(1000.0f);

    float commanded = 0.0f;
    TEST_ASSERT_EQUAL(ESP_OK, speed_profile_update(
        profile, config, 2.0f, 100.0f, 0.1f, commanded));

    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 2.0f, commanded);
    // Landing exactly on the target isn't treated as an overshoot, so the
    // acceleration that produced this cycle's step is retained rather than
    // zeroed; the next cycle's zero speed_error will ramp it down instead.
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 20.0f, profile.commanded_accel_mps2);
}

// Confirms a purely accel-bound ramp (jerk not binding) advances at exactly
// max_accel_mps2 each cycle, matching a simple constant-acceleration ramp.
void test_accel_bound_ramp_matches_constant_acceleration() {
    SpeedProfile profile = {};
    speed_profile_reset(&profile, 0.0f);
    const SpeedProfileConfig config = make_config(1000.0f);  // effectively unbounded jerk

    float commanded = 0.0f;
    for (int step = 1; step <= 5; ++step) {
        TEST_ASSERT_EQUAL(ESP_OK, speed_profile_update(
            profile, config, 10.0f, 1.0f, 0.1f, commanded));
        TEST_ASSERT_FLOAT_WITHIN(kTolerance, 1.0f, profile.commanded_accel_mps2);
        TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.1f * step, commanded);
    }
}

// Confirms a jerk-bound ramp increases acceleration by at most
// max_jerk_mps3 * dt_s each cycle while far from the target.
void test_jerk_limits_acceleration_change_per_cycle() {
    SpeedProfile profile = {};
    speed_profile_reset(&profile, 0.0f);
    const SpeedProfileConfig config = make_config(1.0f);  // tight jerk bound
    constexpr float kDt = 0.1f;
    constexpr float kMaxAccelStep = 1.0f * kDt;  // = max_jerk_mps3 * dt_s

    float commanded = 0.0f;
    float previous_accel = 0.0f;
    for (int step = 1; step <= 4; ++step) {
        TEST_ASSERT_EQUAL(ESP_OK, speed_profile_update(
            profile, config, 10.0f, 100.0f, kDt, commanded));
        TEST_ASSERT_FLOAT_WITHIN(kTolerance, kMaxAccelStep,
                                  profile.commanded_accel_mps2 - previous_accel);
        previous_accel = profile.commanded_accel_mps2;
    }
}

// Confirms the ramp never overshoots a decreasing target and settles exactly
// on it instead of oscillating, across many cycles.
void test_never_overshoots_and_converges() {
    SpeedProfile profile = {};
    speed_profile_reset(&profile, 5.0f);
    const SpeedProfileConfig config = make_config(20.0f);

    float commanded = 5.0f;
    for (int step = 0; step < 200; ++step) {
        const float previous = commanded;
        TEST_ASSERT_EQUAL(ESP_OK, speed_profile_update(
            profile, config, 1.0f, 2.0f, 0.02f, commanded));
        TEST_ASSERT_TRUE(commanded <= previous + kTolerance);
        TEST_ASSERT_TRUE(commanded >= 1.0f - kTolerance);
    }
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 1.0f, commanded);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, profile.commanded_accel_mps2);
}

// Confirms invalid arguments are rejected without mutating the profile.
void test_rejects_invalid_arguments() {
    SpeedProfile profile = {};
    speed_profile_reset(&profile, 0.0f);
    const SpeedProfileConfig config = make_config(1.0f);
    float commanded = 0.0f;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, speed_profile_update(
        profile, config, NAN, 1.0f, 0.1f, commanded));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, speed_profile_update(
        profile, config, 1.0f, 0.0f, 0.1f, commanded));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, speed_profile_update(
        profile, config, 1.0f, 1.0f, 0.0f, commanded));

    const SpeedProfileConfig bad_config = make_config(-1.0f);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, speed_profile_update(
        profile, bad_config, 1.0f, 1.0f, 0.1f, commanded));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_reset_seeds_initial_speed);
    RUN_TEST(test_reaches_target_in_one_cycle_when_unbounded);
    RUN_TEST(test_accel_bound_ramp_matches_constant_acceleration);
    RUN_TEST(test_jerk_limits_acceleration_change_per_cycle);
    RUN_TEST(test_never_overshoots_and_converges);
    RUN_TEST(test_rejects_invalid_arguments);
    return UNITY_END();
}
