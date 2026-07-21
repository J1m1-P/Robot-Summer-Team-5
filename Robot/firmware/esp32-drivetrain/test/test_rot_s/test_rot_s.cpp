/* Tests RotS (open-loop, dead-reckoned in-place rotation) without ESP32 hardware. */
#include <unity.h>

#include <cmath>

#include "control/drivetrain/rot_s.h"

namespace {

constexpr float kTolerance = 1.0e-4f;
constexpr float kPi = 3.14159265358979323846f;

RotSConfig make_config(float angle_tolerance_rad = 0.02f) {
    RotSConfig config = {};
    config.speed_profile.max_jerk_mps3 = 10.0f;
    config.angle_tolerance_rad = angle_tolerance_rad;
    return config;
}

// Keeps test expressions concise while exercising the pointer-based C API.
esp_err_t rot_s_start(
    RotS &rot,
    const RotSConfig &config,
    float start_heading_rad,
    float angle_rad,
    float max_omega_rad_s,
    float max_alpha_rad_s2
) {
    return ::rot_s_start(&rot, &config, start_heading_rad, angle_rad,
                         max_omega_rad_s, max_alpha_rad_s2);
}

esp_err_t rot_s_update(
    RotS &rot,
    float current_heading_rad,
    float dt_s,
    RotSOutput &output
) {
    return ::rot_s_update(&rot, current_heading_rad, dt_s, &output);
}

}  // namespace

void setUp() {}
void tearDown() {}

// Confirms a bad config is rejected.
void test_rejects_invalid_config() {
    RotSConfig config = make_config();
    config.angle_tolerance_rad = -1.0f;
    RotS rot = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, rot_s_start(
        rot, config, 0.0f, kPi / 2.0f, 1.0f, 2.0f));
}

// Confirms start rejects a zero angle and non-positive omega/alpha.
void test_start_rejects_invalid_parameters() {
    const RotSConfig config = make_config();
    RotS rot = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, rot_s_start(
        rot, config, 0.0f, 0.0f, 1.0f, 2.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, rot_s_start(
        rot, config, 0.0f, kPi / 2.0f, 0.0f, 2.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, rot_s_start(
        rot, config, 0.0f, kPi / 2.0f, 1.0f, 0.0f));
}

// Confirms update before start reports invalid state.
void test_update_before_start_is_invalid_state() {
    RotS rot = {};
    RotSOutput output = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, rot_s_update(rot, 0.0f, 0.02f, output));
}

// Confirms a positive (CCW) angle commands positive omega with vx/vy at zero.
void test_positive_angle_commands_positive_omega() {
    const RotSConfig config = make_config();
    RotS rot = {};
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_start(rot, config, 0.0f, kPi / 2.0f, 1.0f, 2.0f));

    RotSOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_update(rot, 0.0f, 0.02f, output));

    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, output.requested_velocity.vx);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, output.requested_velocity.vy);
    TEST_ASSERT_TRUE(output.requested_velocity.omega > 0.0f);
    TEST_ASSERT_EQUAL(static_cast<int>(ROT_S_RUNNING), static_cast<int>(output.status));
}

// Confirms a negative (CW) angle commands negative omega.
void test_negative_angle_commands_negative_omega() {
    const RotSConfig config = make_config();
    RotS rot = {};
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_start(rot, config, 0.0f, -kPi / 2.0f, 1.0f, 2.0f));

    RotSOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_update(rot, 0.0f, 0.02f, output));

    TEST_ASSERT_TRUE(output.requested_velocity.omega < 0.0f);
}

// Confirms progress is measured against the heading captured at start, via
// plain (unwrapped) subtraction -- exercises headings that cross past pi to
// confirm no wraparound logic is silently assumed.
void test_progress_handles_unwrapped_heading_past_pi() {
    const RotSConfig config = make_config();
    RotS rot = {};
    // Start near pi and command a further rotation past it -- the
    // continuously-accumulating heading convention means this is just
    // "3.0 -> 3.3", not a wrap to -pi.
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_start(rot, config, 3.0f, 0.3f, 1.0f, 2.0f));

    RotSOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_update(rot, 3.1f, 0.02f, output));

    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.2f, output.remaining_angle_rad);
}

// Confirms a full simulated run reaches completion within the configured
// tolerance, using a simple Euler integration of the commanded angular
// velocity as a stand-in for a perfectly-tracking wheel-velocity PI. Jerk
// is left effectively unconstrained here so this test isolates RotS's own
// progress-tracking/stopping-distance/completion logic, mirroring
// test_move_s's equivalent simulated-run test.
void test_simulated_run_reaches_target_angle() {
    RotSConfig config = make_config(0.01f);
    config.speed_profile.max_jerk_mps3 = 1000.0f;
    RotS rot = {};
    const float angle_rad = kPi / 2.0f;
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_start(rot, config, 0.0f, angle_rad, 1.0f, 2.0f));

    float heading_rad = 0.0f;
    constexpr float kDt = 0.01f;
    RotSOutput output = {};
    int iterations = 0;
    constexpr int kMaxIterations = 5000;  // 50s simulated cap
    while (output.status != ROT_S_COMPLETE && iterations < kMaxIterations) {
        TEST_ASSERT_EQUAL(ESP_OK, rot_s_update(rot, heading_rad, kDt, output));
        heading_rad += output.requested_velocity.omega * kDt;
        ++iterations;
    }

    TEST_ASSERT_TRUE(iterations < kMaxIterations);
    TEST_ASSERT_EQUAL(static_cast<int>(ROT_S_COMPLETE), static_cast<int>(output.status));
    TEST_ASSERT_TRUE(fabsf(output.remaining_angle_rad) <= config.angle_tolerance_rad + kTolerance);
    TEST_ASSERT_FLOAT_WITHIN(config.angle_tolerance_rad + kTolerance, angle_rad, heading_rad);
}

// Confirms a full simulated run also converges for a negative (CW) target.
void test_simulated_run_reaches_negative_target_angle() {
    RotSConfig config = make_config(0.01f);
    config.speed_profile.max_jerk_mps3 = 1000.0f;
    RotS rot = {};
    const float angle_rad = -kPi / 3.0f;
    TEST_ASSERT_EQUAL(ESP_OK, rot_s_start(rot, config, 0.0f, angle_rad, 1.0f, 2.0f));

    float heading_rad = 0.0f;
    constexpr float kDt = 0.01f;
    RotSOutput output = {};
    int iterations = 0;
    constexpr int kMaxIterations = 5000;
    while (output.status != ROT_S_COMPLETE && iterations < kMaxIterations) {
        TEST_ASSERT_EQUAL(ESP_OK, rot_s_update(rot, heading_rad, kDt, output));
        heading_rad += output.requested_velocity.omega * kDt;
        ++iterations;
    }

    TEST_ASSERT_TRUE(iterations < kMaxIterations);
    TEST_ASSERT_EQUAL(static_cast<int>(ROT_S_COMPLETE), static_cast<int>(output.status));
    TEST_ASSERT_FLOAT_WITHIN(config.angle_tolerance_rad + kTolerance, angle_rad, heading_rad);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_rejects_invalid_config);
    RUN_TEST(test_start_rejects_invalid_parameters);
    RUN_TEST(test_update_before_start_is_invalid_state);
    RUN_TEST(test_positive_angle_commands_positive_omega);
    RUN_TEST(test_negative_angle_commands_negative_omega);
    RUN_TEST(test_progress_handles_unwrapped_heading_past_pi);
    RUN_TEST(test_simulated_run_reaches_target_angle);
    RUN_TEST(test_simulated_run_reaches_negative_target_angle);
    return UNITY_END();
}
