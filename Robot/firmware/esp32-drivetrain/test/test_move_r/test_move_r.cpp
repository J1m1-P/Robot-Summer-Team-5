/* Tests MoveR (closed-loop, absolute-heading in-place rotation) without ESP32 hardware. */
#include <unity.h>

#include <cmath>

#include "control/drivetrain/move_r.h"

namespace {

constexpr float kTolerance = 1.0e-4f;
constexpr float kPi = 3.14159265358979323846f;

MoveRConfig config() {
    MoveRConfig c = {};
    c.heading_controller = {.proportional_gain = 2.0f, .integral_gain = 0.0f,
                             .derivative_gain = 0.0f, .integral_limit = 1.0f,
                             .correction_min = -2.0f, .correction_max = 2.0f};
    c.speed_profile.max_jerk_mps3 = 10.0f;
    c.heading_tolerance_rad = 0.02f;
    c.max_alpha_rad_s2 = 2.0f;
    c.max_omega_rad_s = 1.0f;
    c.controller_dt_max_s = 0.05f;
    return c;
}

}  // namespace

void setUp() {}
void tearDown() {}

// Confirms a bad heading tolerance, angular acceleration ceiling, omega
// ceiling, or dt-fault ceiling is each independently rejected. The last one
// is the field this primitive used to hardcode as a bare 0.05f literal
// instead of reading it from config, unlike every other motion primitive.
void test_rejects_invalid_config() {
    MoveR m = {};
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f, .heading_rad = 0.0f, .valid = true};

    MoveRConfig bad_tolerance = config();
    bad_tolerance.heading_tolerance_rad = -1.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_r_start(&m, &bad_tolerance, &start, 1.0f));

    MoveRConfig bad_alpha = config();
    bad_alpha.max_alpha_rad_s2 = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_r_start(&m, &bad_alpha, &start, 1.0f));

    MoveRConfig bad_omega = config();
    bad_omega.max_omega_rad_s = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_r_start(&m, &bad_omega, &start, 1.0f));

    MoveRConfig bad_dt_max = config();
    bad_dt_max.controller_dt_max_s = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_r_start(&m, &bad_dt_max, &start, 1.0f));
}

// Confirms start rejects an invalid starting estimate.
void test_start_rejects_invalid_estimate() {
    const MoveRConfig c = config();
    MoveR m = {};
    const MotionEstimate bad = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_r_start(&m, &c, &bad, 1.0f));
}

// Confirms update before start is rejected. Unlike MoveP/MoveC/MoveL (which
// check move->config == NULL separately and return ESP_ERR_INVALID_STATE
// without touching status), MoveR folds the never-started case into its
// general argument-validation block, so this returns ESP_ERR_INVALID_ARG and
// marks MOVE_R_FAULT. Documented here as-is rather than "fixed" silently --
// worth reconciling with the other primitives' convention separately.
void test_update_before_start_is_invalid_arg() {
    MoveR m = {};
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f, .heading_rad = 0.0f, .valid = true};
    MoveROutput output = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_r_update(&m, &start, 0.02f, &output));
    TEST_ASSERT_EQUAL(static_cast<int>(MOVE_R_FAULT), static_cast<int>(output.status));
}

// Confirms a target heading ahead of the current one (CCW/positive error)
// commands positive omega with vx/vy held at zero.
void test_positive_heading_error_commands_positive_omega() {
    const MoveRConfig c = config();
    MoveR m = {};
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f, .heading_rad = 0.0f, .valid = true};
    TEST_ASSERT_EQUAL(ESP_OK, move_r_start(&m, &c, &start, kPi / 2.0f));

    MoveROutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_r_update(&m, &start, 0.02f, &output));

    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, output.requested_velocity.vx);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, output.requested_velocity.vy);
    TEST_ASSERT_TRUE(output.requested_velocity.omega > 0.0f);
    TEST_ASSERT_EQUAL(static_cast<int>(MOVE_R_RUNNING), static_cast<int>(output.status));
}

// Confirms a target heading behind the current one commands negative omega.
void test_negative_heading_error_commands_negative_omega() {
    const MoveRConfig c = config();
    MoveR m = {};
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f, .heading_rad = 0.0f, .valid = true};
    TEST_ASSERT_EQUAL(ESP_OK, move_r_start(&m, &c, &start, -kPi / 2.0f));

    MoveROutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_r_update(&m, &start, 0.02f, &output));

    TEST_ASSERT_TRUE(output.requested_velocity.omega < 0.0f);
}

// Regression coverage for the controller_dt_max_s fix: a dt exceeding the
// configured ceiling must fault instead of being silently accepted (or,
// before the fix, checked against a hardcoded 0.05f no caller could tune).
void test_excessive_dt_faults() {
    MoveRConfig c = config();
    c.controller_dt_max_s = 0.05f;
    MoveR m = {};
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f, .heading_rad = 0.0f, .valid = true};
    TEST_ASSERT_EQUAL(ESP_OK, move_r_start(&m, &c, &start, kPi / 2.0f));

    MoveROutput output = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, move_r_update(&m, &start, 0.2f, &output));
    TEST_ASSERT_EQUAL(static_cast<int>(MOVE_R_FAULT), static_cast<int>(output.status));
    TEST_ASSERT_FALSE(output.motion_valid);
}

// Confirms an invalid estimate mid-motion faults with an exact zero command,
// mirroring MoveP/MoveC/MoveL's same contract.
void test_invalid_active_estimate_faults_with_zero_output() {
    const MoveRConfig c = config();
    MoveR m = {};
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f, .heading_rad = 0.0f, .valid = true};
    TEST_ASSERT_EQUAL(ESP_OK, move_r_start(&m, &c, &start, kPi / 2.0f));

    const MotionEstimate bad = {};
    MoveROutput output = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_r_update(&m, &bad, 0.02f, &output));
    TEST_ASSERT_EQUAL(static_cast<int>(MOVE_R_FAULT), static_cast<int>(output.status));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.vy);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.omega);
}

// Confirms a COMPLETE output always carries an exact zero command, never a
// small residual sample from the angular speed profile.
void test_completion_has_exact_zero_command() {
    const MoveRConfig c = config();
    MoveR m = {};
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f, .heading_rad = 0.0f, .valid = true};
    TEST_ASSERT_EQUAL(ESP_OK, move_r_start(&m, &c, &start, 0.0f));

    // Already at the target heading with the profile at rest: one cycle
    // should be enough to latch completion.
    MoveROutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_r_update(&m, &start, 0.02f, &output));
    TEST_ASSERT_EQUAL(static_cast<int>(MOVE_R_COMPLETE), static_cast<int>(output.status));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.vy);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.omega);
}

// Full simulated run: manually integrates heading from the commanded omega
// each cycle (there is no odometry here, just MoveR's own closed-loop
// feedback against a moving MotionEstimate) and confirms it reaches the
// target within tolerance and settles to a stopped, complete state.
void test_simulated_run_reaches_target_heading() {
    MoveRConfig c = config();
    c.speed_profile.max_jerk_mps3 = 1000.0f;
    MoveR m = {};
    MotionEstimate estimate = {.x_m = 0.0f, .y_m = 0.0f, .heading_rad = 0.0f, .valid = true};
    const float target_rad = kPi / 3.0f;
    TEST_ASSERT_EQUAL(ESP_OK, move_r_start(&m, &c, &estimate, target_rad));

    constexpr float kDt = 0.01f;
    MoveROutput output = {};
    int iterations = 0;
    constexpr int kMaxIterations = 5000;  // 50s simulated cap
    while (output.status != MOVE_R_COMPLETE && iterations < kMaxIterations) {
        TEST_ASSERT_EQUAL(ESP_OK, move_r_update(&m, &estimate, kDt, &output));
        estimate.heading_rad += output.requested_velocity.omega * kDt;
        ++iterations;
    }

    TEST_ASSERT_TRUE(iterations < kMaxIterations);
    TEST_ASSERT_EQUAL(static_cast<int>(MOVE_R_COMPLETE), static_cast<int>(output.status));
    TEST_ASSERT_TRUE(fabsf(output.heading_error_rad) <= c.heading_tolerance_rad + kTolerance);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.omega);
}

// Confirms the shortest-path wrap holds across the +-pi seam: starting just
// short of +pi with a target just past -pi (the same physical heading
// region, opposite sides of the wrap point) must turn a small amount in one
// direction, never spin the long way around.
void test_wraps_shortest_path_across_pi_boundary() {
    const MoveRConfig c = config();
    MoveR m = {};
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f, .heading_rad = 3.0f, .valid = true};
    TEST_ASSERT_EQUAL(ESP_OK, move_r_start(&m, &c, &start, -3.0f));

    MoveROutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_r_update(&m, &start, 0.02f, &output));

    TEST_ASSERT_TRUE(fabsf(output.heading_error_rad) < 1.0f);
    TEST_ASSERT_TRUE(output.requested_velocity.omega > 0.0f);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_rejects_invalid_config);
    RUN_TEST(test_start_rejects_invalid_estimate);
    RUN_TEST(test_update_before_start_is_invalid_arg);
    RUN_TEST(test_positive_heading_error_commands_positive_omega);
    RUN_TEST(test_negative_heading_error_commands_negative_omega);
    RUN_TEST(test_excessive_dt_faults);
    RUN_TEST(test_invalid_active_estimate_faults_with_zero_output);
    RUN_TEST(test_completion_has_exact_zero_command);
    RUN_TEST(test_simulated_run_reaches_target_heading);
    RUN_TEST(test_wraps_shortest_path_across_pi_boundary);
    return UNITY_END();
}
