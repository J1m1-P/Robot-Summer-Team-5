/* Tests MoveL's source-agnostic closed-loop path contract. */
#include <unity.h>

#include <cmath>

#include "control/drivetrain/move_l.h"

namespace {

constexpr float kTolerance = 1.0e-4f;
constexpr float kPi = 3.14159265358979323846f;

MoveLConfig make_config() {
    MoveLConfig config = {};
    config.off_tape_motion.controller.proportional_gain = 1.0f;
    config.off_tape_motion.controller.integral_gain = 0.0f;
    config.off_tape_motion.controller.derivative_gain = 0.0f;
    config.off_tape_motion.controller.integral_limit = 1.0f;
    config.off_tape_motion.controller.correction_min = -0.5f;
    config.off_tape_motion.controller.correction_max = 0.5f;
    config.off_tape_motion.controller_dt_max_s = 0.05f;
    config.speed_profile.max_jerk_mps3 = 10.0f;
    config.distance_tolerance_m = 0.01f;
    config.max_accel_mps2 = 1.0f;
    return config;
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_rejects_invalid_config() {
    MoveLConfig config = make_config();
    config.max_accel_mps2 = 0.0f;
    MoveL move = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        move_l_start(&move, &config, 1.0f, 0.0f, 0.5f));
}

void test_update_before_start_is_invalid_state() {
    MoveL move = {};
    MoveLInput input = {};
    MoveLOutput output = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
        move_l_update(&move, &input, 0.02f, &output));
}

// The input is already path error/progress; the primitive has no knowledge of
// whether an encoder estimator or a PMW3610 estimator supplied it.
void test_applies_source_agnostic_cross_track_correction() {
    const MoveLConfig config = make_config();
    MoveL move = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_l_start(&move, &config, 1.0f, 0.0f, 0.5f));

    const MoveLInput input = {.along_track_progress_m = 0.0f, .cross_track_error_m = 0.1f, .valid = true};
    MoveLOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_l_update(&move, &input, 0.02f, &output));

    TEST_ASSERT_TRUE(output.motion_valid);
    TEST_ASSERT_TRUE(output.requested_velocity.vx > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.1f, output.requested_velocity.vy);
}

void test_rotates_path_frame_command_into_requested_body_heading() {
    const MoveLConfig config = make_config();
    MoveL move = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_l_start(&move, &config, 1.0f, kPi / 2.0f, 0.5f));

    const MoveLInput input = {.along_track_progress_m = 0.0f, .cross_track_error_m = 0.1f, .valid = true};
    MoveLOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_l_update(&move, &input, 0.02f, &output));

    TEST_ASSERT_FLOAT_WITHIN(kTolerance, -0.1f, output.requested_velocity.vx);
    TEST_ASSERT_TRUE(output.requested_velocity.vy > 0.0f);
}

void test_reaches_completion_from_external_along_track_progress() {
    const MoveLConfig config = make_config();
    MoveL move = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_l_start(&move, &config, 1.0f, 0.0f, 0.5f));

    MoveLInput input = {};
    input.valid = true;
    MoveLOutput output = {};
    int iterations = 0;
    while (output.status != MOVE_L_COMPLETE && iterations < 5000) {
        TEST_ASSERT_EQUAL(ESP_OK, move_l_update(&move, &input, 0.01f, &output));
        input.along_track_progress_m += output.requested_velocity.vx * 0.01f;
        ++iterations;
    }

    TEST_ASSERT_TRUE(iterations < 5000);
    TEST_ASSERT_EQUAL(static_cast<int>(MOVE_L_COMPLETE), static_cast<int>(output.status));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.vy);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.omega);
}

void test_invalid_feedback_faults_with_zero_output() {
    const MoveLConfig config = make_config();
    MoveL move = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_l_start(&move, &config, 1.0f, 0.0f, 0.5f));
    const MoveLInput invalid = {.valid = false};
    MoveLOutput output = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_l_update(&move, &invalid, 0.02f, &output));
    TEST_ASSERT_EQUAL(MOVE_L_FAULT, output.status);
    TEST_ASSERT_FALSE(output.motion_valid);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, output.requested_velocity.vy);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_rejects_invalid_config);
    RUN_TEST(test_update_before_start_is_invalid_state);
    RUN_TEST(test_applies_source_agnostic_cross_track_correction);
    RUN_TEST(test_rotates_path_frame_command_into_requested_body_heading);
    RUN_TEST(test_reaches_completion_from_external_along_track_progress);
    RUN_TEST(test_invalid_feedback_faults_with_zero_output);
    return UNITY_END();
}
