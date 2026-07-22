/* Tests pure source-agnostic path-planning geometry. */
#include <unity.h>

#include <cmath>

#include "control/drivetrain/path_planner.h"

namespace {
constexpr float kTolerance = 1.0e-4f;
}

void setUp() {}
void tearDown() {}

void test_builds_shortest_line_from_estimate_to_target() {
    const MotionEstimate start = {.x_m = 1.0f, .y_m = 2.0f, .heading_rad = 0.0f, .valid = true};
    PathPlannerLine line = {};
    TEST_ASSERT_EQUAL(ESP_OK, path_planner_line_start(&line, &start, 4.0f, 6.0f));
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 5.0f, line.length_m);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.6f, line.direction_x);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.8f, line.direction_y);
}

void test_reports_path_progress_and_correction_direction() {
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f, .heading_rad = 0.0f, .valid = true};
    PathPlannerLine line = {};
    TEST_ASSERT_EQUAL(ESP_OK, path_planner_line_start(&line, &start, 1.0f, 0.0f));

    const MotionEstimate estimate = {.x_m = 0.4f, .y_m = -0.2f, .heading_rad = 0.0f, .valid = true};
    PathPlannerLineFeedback feedback = {};
    TEST_ASSERT_EQUAL(ESP_OK, path_planner_line_feedback(&line, &estimate, &feedback));
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.4f, feedback.along_track_progress_m);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.2f, feedback.correction_error_m);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, hypotf(0.6f, 0.2f), feedback.distance_to_goal_m);
}

void test_rejects_invalid_estimate() {
    MotionEstimate invalid = {.x_m = 0.0f, .y_m = 0.0f, .heading_rad = 0.0f, .valid = false};
    PathPlannerLine line = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, path_planner_line_start(&line, &invalid, 1.0f, 0.0f));
}

void test_wraps_very_large_finite_angle_without_iteration() {
    const float wrapped = path_planner_wrap_angle_rad(1000000000.0f);
    TEST_ASSERT_TRUE(std::isfinite(wrapped));
    TEST_ASSERT_TRUE(wrapped >= -3.1415927f && wrapped <= 3.1415927f);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_builds_shortest_line_from_estimate_to_target);
    RUN_TEST(test_reports_path_progress_and_correction_direction);
    RUN_TEST(test_rejects_invalid_estimate);
    RUN_TEST(test_wraps_very_large_finite_angle_without_iteration);
    return UNITY_END();
}
