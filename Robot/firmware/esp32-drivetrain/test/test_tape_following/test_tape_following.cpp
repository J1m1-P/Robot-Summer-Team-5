#include <math.h>
#include <string.h>

#include <unity.h>

extern "C" {
#include "control/tape_following/tape_follower.h"
#include "control/tape_following/tape_following_controller.h"
#include "control/tape_following/tape_following_kinematics.h"
#include "sensing/tape_following/tape_line_estimator.h"
#include "sensing/tape_following/tape_task_detection.h"
}

void setUp() {}
void tearDown() {}

static TapeSensor make_sensor(bool ch0, bool ch1, bool ch2, bool ch3)
{
    TapeSensor sensor = {};
    sensor.channel_0 = ch0;
    sensor.channel_1 = ch1;
    sensor.channel_2 = ch2;
    sensor.channel_3 = ch3;
    return sensor;
}

static TapeLineEstimatorConfig make_estimator_config()
{
    TapeLineEstimatorConfig config = {{-3.0f, -1.0f, 1.0f, 3.0f}};
    return config;
}

static TapeFollowerConfig make_follower_config(
    const TapeLineEstimatorConfig *front,
    const TapeLineEstimatorConfig *back)
{
    TapeFollowerConfig config = {};
    config.estimators[TAPE_FOLLOWER_FRONT] = front;
    config.estimators[TAPE_FOLLOWER_BACK] = back;
    config.controller.proportional_gain = 0.1f;
    config.controller.integral_limit = 1.0f;
    config.controller.correction_min = -0.3f;
    config.controller.correction_max = 0.3f;
    config.heading.gain_s_inv = 2.0f;
    config.heading.max_omega_rad_s = 0.8f;
    config.heading.max_acceleration_rad_s2 = 1.5f;
    config.search.angular_velocity_rad_s = 0.4f;
    config.search.timeout_s = 0.5f;
    config.controller_dt_max_s = 0.05f;
    return config;
}

static void test_line_estimator_centroid_and_lost_direction()
{
    TapeLineEstimatorConfig config = make_estimator_config();
    TapeLineEstimatorState state = {};
    TapeSensor sensor = make_sensor(false, false, true, true);
    float error = 0.0f;

    TEST_ASSERT_TRUE(tape_line_estimator_compute_error(
        &sensor, &config, &state, &error));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 2.0f, error);

    sensor = make_sensor(false, false, false, false);
    TEST_ASSERT_FALSE(tape_line_estimator_compute_error(
        &sensor, &config, &state, &error));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 3.0f, error);
}

static void test_line_estimator_0110_is_centered()
{
    TapeLineEstimatorConfig config = make_estimator_config();
    TapeLineEstimatorState state = {};
    TapeSensor sensor = make_sensor(false, true, true, false);
    float error = 99.0f;

    TEST_ASSERT_TRUE(tape_line_estimator_compute_error(
        &sensor, &config, &state, &error));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, error);
}

static void test_controller_clamps_and_rejects_invalid_input()
{
    TapeFollowingControllerConfig config = {
        1.0f, 0.5f, 0.0f, 0.2f, -0.3f, 0.3f};
    TapeFollowingControllerState state = {};

    TEST_ASSERT_TRUE(tape_following_controller_config_is_valid(&config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_following_controller_reset(&state));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      tape_following_controller_reset(nullptr));

    float correction = tape_following_controller_update(
        &state, &config, 2.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.3f, correction);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.2f, state.integral);

    correction = tape_following_controller_update(
        &state, &config, NAN, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, correction);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.2f, state.integral);

    config.correction_min = 1.0f;
    config.correction_max = -1.0f;
    TEST_ASSERT_FALSE(tape_following_controller_config_is_valid(&config));
}

static void test_line_estimator_rejects_non_finite_weights()
{
    TapeLineEstimatorConfig config = make_estimator_config();
    config.channel_weights[1] = NAN;
    TapeLineEstimatorState state = {};
    TapeSensor sensor = make_sensor(false, true, false, false);
    float error = 42.0f;

    TEST_ASSERT_FALSE(tape_line_estimator_config_is_valid(&config));
    TEST_ASSERT_FALSE(tape_line_estimator_compute_error(
        &sensor, &config, &state, &error));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 42.0f, error);
}

static void test_follower_outputs_drivetrain_body_velocity()
{
    TapeLineEstimatorConfig front_config = make_estimator_config();
    TapeLineEstimatorConfig back_config = make_estimator_config();
    TapeFollowerConfig config = make_follower_config(
        &front_config, &back_config);
    TapeFollower follower = {};
    TapeSensor front = make_sensor(false, false, true, false);
    TapeSensor back = make_sensor(false, true, false, false);
    TapeFollowerInput input = {{&front, &back}, 0.4f};
    TapeFollowerOutput output = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_init(&follower, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.01f, &output));
    TEST_ASSERT_TRUE(output.motion_valid);
    TEST_ASSERT_EQUAL(TAPE_FOLLOWER_TRACKING, output.status);
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.4f, output.requested_velocity.vx);
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.1f, output.requested_velocity.vy);
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, -0.015f, output.requested_velocity.omega);

    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.01f, &output));
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, -0.030f, output.requested_velocity.omega);
}

static void test_kinematics_turns_leading_edge_and_limits_acceleration()
{
    const TapeFollowingKinematicsConfig config = {2.0f, 0.8f, 1.5f};
    float omega = 0.0f;

    TEST_ASSERT_EQUAL(ESP_OK,
        tape_following_kinematics_velocity_to_angular_velocity(
            &config, 0.4f, 0.1f, 0.0f, 0.1f, &omega));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.15f, omega);

    TEST_ASSERT_EQUAL(ESP_OK,
        tape_following_kinematics_velocity_to_angular_velocity(
            &config, -0.4f, 0.1f, 0.0f, 0.1f, &omega));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.15f, omega);
}

static void test_follower_turns_back_sensor_toward_travel()
{
    TapeLineEstimatorConfig front_config = make_estimator_config();
    TapeLineEstimatorConfig back_config = make_estimator_config();
    TapeFollowerConfig config = make_follower_config(
        &front_config, &back_config);
    TapeFollower follower = {};
    TapeSensor front = {};
    TapeSensor back = make_sensor(false, false, true, false);
    TapeFollowerInput input = {{&front, &back}, -0.4f};
    TapeFollowerOutput output = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_init(&follower, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.1f, &output));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.4f, output.requested_velocity.vx);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1f, output.requested_velocity.vy);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.075f, output.requested_velocity.omega);
}

static void test_follower_searches_then_reports_lost()
{
    TapeLineEstimatorConfig front_config = make_estimator_config();
    TapeLineEstimatorConfig back_config = make_estimator_config();
    TapeFollowerConfig config = make_follower_config(
        &front_config, &back_config);
    TapeFollower follower = {};
    TapeSensor front = make_sensor(false, false, true, false);
    TapeSensor back = {};
    TapeFollowerInput input = {{&front, &back}, 0.4f};
    TapeFollowerOutput output = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_init(&follower, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.01f, &output));

    front = {};
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.1f, &output));
    TEST_ASSERT_EQUAL(TAPE_FOLLOWER_SEARCHING, output.status);
    TEST_ASSERT_TRUE(output.motion_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, output.requested_velocity.vx);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, output.requested_velocity.vy);
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, -0.4f, output.requested_velocity.omega);

    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.4f, &output));
    TEST_ASSERT_EQUAL(TAPE_FOLLOWER_LOST, output.status);
    TEST_ASSERT_FALSE(output.motion_valid);
}

static void test_follower_search_preserves_reversed_front_last_side()
{
    TapeLineEstimatorConfig front_config = {{3.0f, 1.0f, -1.0f, -3.0f}};
    TapeLineEstimatorConfig back_config = make_estimator_config();
    TapeFollowerConfig config = make_follower_config(
        &front_config, &back_config);
    TapeFollower follower = {};
    /* On the reversed front module, channel 3 is the physical left edge. */
    TapeSensor front = make_sensor(false, false, false, true);
    TapeSensor back = {};
    TapeFollowerInput input = {{&front, &back}, 0.4f};
    TapeFollowerOutput output = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_init(&follower, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.01f, &output));
    TEST_ASSERT_EQUAL(TAPE_FOLLOWER_TRACKING, output.status);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -3.0f, output.line_error);

    front = {};
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.1f, &output));
    TEST_ASSERT_EQUAL(TAPE_FOLLOWER_SEARCHING, output.status);
    TEST_ASSERT_TRUE(output.motion_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, output.requested_velocity.vx);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, output.requested_velocity.vy);
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.4f, output.requested_velocity.omega);
}

static void test_follower_search_mirrors_turn_for_reverse_travel()
{
    TapeLineEstimatorConfig front_config = make_estimator_config();
    TapeLineEstimatorConfig back_config = make_estimator_config();
    TapeFollowerConfig config = make_follower_config(
        &front_config, &back_config);
    TapeFollower follower = {};
    TapeSensor front = {};
    TapeSensor back = make_sensor(false, false, true, false);
    TapeFollowerInput input = {{&front, &back}, -0.4f};
    TapeFollowerOutput output = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_init(&follower, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.01f, &output));

    back = {};
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.1f, &output));
    TEST_ASSERT_EQUAL(TAPE_FOLLOWER_SEARCHING, output.status);
    TEST_ASSERT_TRUE(output.motion_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, output.requested_velocity.vx);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, output.requested_velocity.vy);
    TEST_ASSERT_FLOAT_WITHIN(
        0.0001f, 0.4f, output.requested_velocity.omega);
}

static void test_task_detector_debounces_start_and_end()
{
    TapeTaskDetectorConfig config = {2, 2, 2};
    TapeTaskDetector detector = {};
    TapeTaskDetectionOutput output = {};
    TapeSensor active = make_sensor(true, true, false, false);
    TapeSensor inactive = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_task_detector_init(&detector, &config));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      tape_task_detector_init(&detector, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_task_detector_update(
        &detector, &active, &output));
    TEST_ASSERT_FALSE(output.detected);
    TEST_ASSERT_EQUAL(ESP_OK, tape_task_detector_update(
        &detector, &active, &output));
    TEST_ASSERT_TRUE(output.detected);
    TEST_ASSERT_TRUE(output.detection_started);

    TEST_ASSERT_EQUAL(ESP_OK, tape_task_detector_update(
        &detector, &inactive, &output));
    TEST_ASSERT_TRUE(output.detected);
    TEST_ASSERT_EQUAL(ESP_OK, tape_task_detector_update(
        &detector, &inactive, &output));
    TEST_ASSERT_FALSE(output.detected);
    TEST_ASSERT_TRUE(output.detection_ended);
}

int main(int, char **)
{
    UNITY_BEGIN();
    RUN_TEST(test_line_estimator_centroid_and_lost_direction);
    RUN_TEST(test_line_estimator_0110_is_centered);
    RUN_TEST(test_line_estimator_rejects_non_finite_weights);
    RUN_TEST(test_controller_clamps_and_rejects_invalid_input);
    RUN_TEST(test_kinematics_turns_leading_edge_and_limits_acceleration);
    RUN_TEST(test_follower_outputs_drivetrain_body_velocity);
    RUN_TEST(test_follower_turns_back_sensor_toward_travel);
    RUN_TEST(test_follower_searches_then_reports_lost);
    RUN_TEST(test_follower_search_preserves_reversed_front_last_side);
    RUN_TEST(test_follower_search_mirrors_turn_for_reverse_travel);
    RUN_TEST(test_task_detector_debounces_start_and_end);
    return UNITY_END();
}
