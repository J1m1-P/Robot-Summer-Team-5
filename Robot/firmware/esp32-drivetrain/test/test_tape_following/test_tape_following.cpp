#include <math.h>
#include <string.h>

#include <unity.h>

extern "C" {
#include "control/tape_following/tape_follower.h"
#include "control/tape_following/tape_alignment.h"
#include "control/tape_following/tape_following_controller.h"
#include "control/tape_following/tape_following_kinematics.h"
#include "sensing/tape_following/tape_line_estimator.h"
#include "sensing/tape_following/tape_locating_detection.h"
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
    config.estimators[TAPE_FOLLOWER_SIDE] = front;
    config.lateral_motion.controller.proportional_gain = 0.1f;
    config.lateral_motion.controller.integral_limit = 1.0f;
    config.lateral_motion.controller.correction_min = -0.3f;
    config.lateral_motion.controller.correction_max = 0.3f;
    config.heading.gain_s_inv = 2.0f;
    config.heading.max_omega_rad_s = 0.8f;
    config.heading.max_acceleration_rad_s2 = 1.5f;
    config.search.angular_velocity_rad_s = 0.4f;
    config.search.timeout_s = 0.5f;
    config.lateral_motion.controller_dt_max_s = 0.05f;
    // High enough that it never actually engages within a single test call,
    // so existing single-step assertions built against the pre-slew-limit
    // behavior stay valid. See test_lateral_velocity_is_slew_rate_limited
    // for a config that deliberately sets this tight.
    config.max_lateral_accel_mps2 = 100.0f;
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
    TapeFollowerInput input = {{&front, &back, nullptr},
                               TAPE_FOLLOWER_PX, 0.4f, 0.0f};
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

static void test_follower_slew_limits_lateral_velocity()
{
    TapeLineEstimatorConfig front_config = make_estimator_config();
    TapeLineEstimatorConfig back_config = make_estimator_config();
    TapeFollowerConfig config = make_follower_config(
        &front_config, &back_config);
    // Deliberately tight: the sensor input below drives the PID straight to
    // correction_max (0.3), but the slew limit should only let the output
    // climb 0.01 m/s per 0.01s cycle instead of jumping there instantly.
    config.max_lateral_accel_mps2 = 1.0f;
    TapeFollower follower = {};
    TapeSensor front = make_sensor(false, false, false, true);  // weight=3.0
    TapeSensor back = make_sensor(false, false, false, true);
    TapeFollowerInput input = {{&front, &back, nullptr},
                               TAPE_FOLLOWER_PX, 0.4f, 0.0f};
    TapeFollowerOutput output = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_init(&follower, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.01f, &output));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.01f, output.requested_velocity.vy);

    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.01f, &output));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.02f, output.requested_velocity.vy);

    // Many cycles later, it should have caught up to the PID's actual
    // (clamped) target rather than being permanently capped.
    for (int i = 0; i < 100; ++i) {
        TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
            &follower, &input, 0.01f, &output));
    }
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.3f, output.requested_velocity.vy);
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
    TapeFollowerInput input = {{&front, &back, nullptr},
                               TAPE_FOLLOWER_MX, 0.4f, 0.0f};
    TapeFollowerOutput output = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_init(&follower, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.1f, &output));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.4f, output.requested_velocity.vx);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.1f, output.requested_velocity.vy);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.075f, output.requested_velocity.omega);
}

static void test_follower_tracks_py_with_rotated_px_polarity()
{
    TapeLineEstimatorConfig front_config = make_estimator_config();
    TapeLineEstimatorConfig back_config = make_estimator_config();
    TapeFollowerConfig config = make_follower_config(
        &front_config, &back_config);
    TapeFollower follower = {};
    TapeSensor front = {};
    TapeSensor back = {};
    TapeSensor side = make_sensor(false, false, true, false);
    TapeFollowerInput input = {{&front, &back, &side},
                               TAPE_FOLLOWER_PY, 0.4f, 0.0f};
    TapeFollowerOutput output = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_init(&follower, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.01f, &output));
    TEST_ASSERT_EQUAL(TAPE_FOLLOWER_TRACKING, output.status);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.1f,
                             output.requested_velocity.vx);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.4f,
                             output.requested_velocity.vy);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.015f,
                             output.requested_velocity.omega);
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
    TapeFollowerInput input = {{&front, &back, nullptr},
                               TAPE_FOLLOWER_PX, 0.4f, 0.0f};
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
    TapeFollowerInput input = {{&front, &back, nullptr},
                               TAPE_FOLLOWER_PX, 0.4f, 0.0f};
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
    TapeFollowerInput input = {{&front, &back, nullptr},
                               TAPE_FOLLOWER_MX, 0.4f, 0.0f};
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

static void test_follower_tracks_signed_net_along_tape_progress()
{
    TapeLineEstimatorConfig front_config = make_estimator_config();
    TapeLineEstimatorConfig back_config = make_estimator_config();
    TapeFollowerConfig config = make_follower_config(
        &front_config, &back_config);
    TapeFollower follower = {};
    TapeSensor front = make_sensor(false, true, true, false);
    TapeSensor back = {};
    TapeFollowerInput input = {{&front, &back, nullptr},
                               TAPE_FOLLOWER_PX, 0.4f, 0.10f};
    TapeFollowerOutput output = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_init(&follower, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.01f, &output));
    input.along_tape_delta_m = 0.12f;
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.01f, &output));
    /* A small backwards correction reduces net progress; it is not converted
     * into extra path length by taking an absolute value. */
    input.along_tape_delta_m = -0.03f;
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.01f, &output));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.19f,
                             output.along_tape_distance_m);

    input.travel_velocity_mps = 0.0f;
    input.along_tape_delta_m = 1.0f;
    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_update(
        &follower, &input, 0.01f, &output));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.19f,
                             output.along_tape_distance_m);

    TEST_ASSERT_EQUAL(ESP_OK, tape_follower_reset(&follower));
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f, follower.along_tape_distance_m);
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

static TapeLocatingDetectorConfig make_locating_config()
{
    TapeLocatingDetectorConfig config = {};
    config.locating_side = TAPE_LOCATING_CCW;
    config.expected_marker = TAPE_LOCATING_MARKER_DOUBLE;
    config.minimum_active_channels = 1;
    config.confirmation_samples = 1;
    config.release_samples = 1;
    config.tape_width_m = 0.01905f;
    config.double_center_distance_m = 0.045f;
    config.spacing_tolerance_m = 0.005f;
    return config;
}

static void test_locating_detector_recognizes_two_piece_marker()
{
    TapeLocatingDetectorConfig config = make_locating_config();
    TapeLocatingDetector detector = {};
    TapeLocatingDetectionOutput output = {};
    TapeSensor active = make_sensor(true, false, false, false);
    TapeSensor inactive = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_locating_detector_init(&detector, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_locating_detector_update(
        &detector, &active, 0.0f, &output));
    TEST_ASSERT_EQUAL(ESP_OK, tape_locating_detector_update(
        &detector, &inactive, 0.010f, &output));
    TEST_ASSERT_EQUAL(ESP_OK, tape_locating_detector_update(
        &detector, &active, 0.035f, &output));
    TEST_ASSERT_EQUAL(ESP_OK, tape_locating_detector_update(
        &detector, &inactive, 0.010f, &output));
    TEST_ASSERT_TRUE(output.event);
    TEST_ASSERT_EQUAL(TAPE_LOCATING_MARKER_DOUBLE, output.marker);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0275f,
                             output.marker_center_progress_m);

    TEST_ASSERT_EQUAL(ESP_OK, tape_locating_detector_update(
        &detector, &active, 0.050f, &output));
    TEST_ASSERT_FALSE(output.event);
}

static void test_locating_detector_recognizes_configured_single_marker()
{
    TapeLocatingDetectorConfig config = make_locating_config();
    config.expected_marker = TAPE_LOCATING_MARKER_SINGLE;
    TapeLocatingDetector detector = {};
    TapeLocatingDetectionOutput output = {};
    TapeSensor active = make_sensor(true, false, false, false);
    TapeSensor inactive = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_locating_detector_init(&detector, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_locating_detector_update(
        &detector, &active, 0.0f, &output));
    TEST_ASSERT_EQUAL(ESP_OK, tape_locating_detector_update(
        &detector, &inactive, 0.010f, &output));
    TEST_ASSERT_TRUE(output.event);
    TEST_ASSERT_EQUAL(TAPE_LOCATING_MARKER_SINGLE, output.marker);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.005f,
                             output.marker_center_progress_m);
}

static TapeAlignmentConfig make_alignment_config(
    TapeAlignmentMode mode,
    const TapeLineEstimatorConfig *estimator)
{
    TapeAlignmentConfig config = {};
    config.mode = mode;
    config.estimators[TAPE_FOLLOWER_FRONT] = estimator;
    config.estimators[TAPE_FOLLOWER_BACK] = estimator;
    config.estimators[TAPE_FOLLOWER_SIDE] = estimator;
    config.correction_speed_mps = 0.08f;
    config.error_tolerance = 0.25f;
    config.timeout_s = 1.0f;
    config.settle_samples = 2;
    return config;
}

static void test_i_align_completes_after_stable_front_back_alignment()
{
    TapeLineEstimatorConfig estimator = make_estimator_config();
    TapeAlignmentConfig config = make_alignment_config(
        TAPE_ALIGNMENT_I_ALIGN_LONGITUDINAL, &estimator);
    TapeAlignment alignment = {};
    TapeSensor front = make_sensor(false, true, true, false);
    TapeSensor back = make_sensor(false, true, true, false);
    TapeAlignmentInput input = {{&front, &back, nullptr}, 0.01f};
    TapeAlignmentOutput output = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_alignment_init(&alignment, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_alignment_start(&alignment));
    TEST_ASSERT_EQUAL(ESP_OK, tape_alignment_update(
        &alignment, &input, &output));
    TEST_ASSERT_EQUAL(TAPE_ALIGNMENT_RUNNING, output.status);
    TEST_ASSERT_TRUE(output.aligned);
    TEST_ASSERT_FALSE(output.motion_valid);
    TEST_ASSERT_EQUAL(ESP_OK, tape_alignment_update(
        &alignment, &input, &output));
    TEST_ASSERT_EQUAL(TAPE_ALIGNMENT_COMPLETE, output.status);
    TEST_ASSERT_FALSE(output.motion_valid);
}

static void test_i_align_uses_bounded_lateral_correction()
{
    TapeLineEstimatorConfig estimator = make_estimator_config();
    TapeAlignmentConfig config = make_alignment_config(
        TAPE_ALIGNMENT_I_ALIGN_LONGITUDINAL, &estimator);
    TapeAlignment alignment = {};
    TapeSensor front = make_sensor(false, false, true, false);
    TapeSensor back = make_sensor(false, false, true, false);
    TapeAlignmentInput input = {{&front, &back, nullptr}, 0.01f};
    TapeAlignmentOutput output = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_alignment_init(&alignment, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_alignment_start(&alignment));
    TEST_ASSERT_EQUAL(ESP_OK, tape_alignment_update(
        &alignment, &input, &output));
    TEST_ASSERT_TRUE(output.motion_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.08f,
                             output.requested_velocity.vy);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f,
                             output.requested_velocity.vx);
}

static void test_l_align_corrects_lateral_overshoot_without_longitudinal_motion()
{
    TapeLineEstimatorConfig estimator = make_estimator_config();
    TapeAlignmentConfig config = make_alignment_config(
        TAPE_ALIGNMENT_L_ALIGN_PY_MX, &estimator);
    TapeAlignment alignment = {};
    TapeSensor back = make_sensor(false, true, true, false);
    TapeSensor side = make_sensor(false, false, true, false);
    TapeAlignmentInput input = {{nullptr, &back, &side}, 0.01f};
    TapeAlignmentOutput output = {};

    TEST_ASSERT_EQUAL(ESP_OK, tape_alignment_init(&alignment, &config));
    TEST_ASSERT_EQUAL(ESP_OK, tape_alignment_start(&alignment));
    TEST_ASSERT_EQUAL(ESP_OK, tape_alignment_update(
        &alignment, &input, &output));
    TEST_ASSERT_TRUE(output.motion_valid);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, -0.08f,
                             output.requested_velocity.vx);
    TEST_ASSERT_FLOAT_WITHIN(0.0001f, 0.0f,
                             output.requested_velocity.vy);
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
    RUN_TEST(test_follower_slew_limits_lateral_velocity);
    RUN_TEST(test_follower_turns_back_sensor_toward_travel);
    RUN_TEST(test_follower_tracks_py_with_rotated_px_polarity);
    RUN_TEST(test_follower_searches_then_reports_lost);
    RUN_TEST(test_follower_search_preserves_reversed_front_last_side);
    RUN_TEST(test_follower_search_mirrors_turn_for_reverse_travel);
    RUN_TEST(test_follower_tracks_signed_net_along_tape_progress);
    RUN_TEST(test_task_detector_debounces_start_and_end);
    RUN_TEST(test_locating_detector_recognizes_two_piece_marker);
    RUN_TEST(test_locating_detector_recognizes_configured_single_marker);
    RUN_TEST(test_i_align_completes_after_stable_front_back_alignment);
    RUN_TEST(test_i_align_uses_bounded_lateral_correction);
    RUN_TEST(test_l_align_corrects_lateral_overshoot_without_longitudinal_motion);
    return UNITY_END();
}
