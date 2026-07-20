/* Tests MoveS (open-loop, dead-reckoned straight-line move) without ESP32 hardware. */
#include <unity.h>

#include <cmath>

#include "control/drivetrain/move_s.h"

namespace {

constexpr float kTolerance = 1.0e-4f;
constexpr float kPi = 3.14159265358979323846f;

MoveSConfig make_config(float distance_tolerance_m = 0.01f) {
    MoveSConfig config = {};
    config.speed_profile.max_jerk_mps3 = 10.0f;
    config.distance_tolerance_m = distance_tolerance_m;
    return config;
}

DrivetrainPose make_pose(float x_mm, float y_mm, float heading_rad) {
    DrivetrainPose pose = {};
    pose.x_mm = x_mm;
    pose.y_mm = y_mm;
    pose.heading_rad = heading_rad;
    return pose;
}

// Keeps test expressions concise while exercising the pointer-based C API.
esp_err_t move_s_start(
    MoveS &move,
    const MoveSConfig &config,
    const DrivetrainPose &start_pose,
    float distance_m,
    float heading_rad,
    float max_speed_mps,
    float max_accel_mps2
) {
    return ::move_s_start(&move, &config, &start_pose, distance_m, heading_rad,
                          max_speed_mps, max_accel_mps2);
}

esp_err_t move_s_update(
    MoveS &move,
    const DrivetrainPose &current_pose,
    float dt_s,
    MoveSOutput &output
) {
    return ::move_s_update(&move, &current_pose, dt_s, &output);
}

}  // namespace

void setUp() {}
void tearDown() {}

// Confirms a bad config is rejected.
void test_rejects_invalid_config() {
    MoveSConfig config = make_config();
    config.distance_tolerance_m = -1.0f;
    MoveS move = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_s_start(
        move, config, make_pose(0, 0, 0), 1.0f, 0.0f, 0.3f, 1.0f));
}

// Confirms start rejects non-positive distance/speed/accel.
void test_start_rejects_invalid_parameters() {
    const MoveSConfig config = make_config();
    MoveS move = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_s_start(
        move, config, make_pose(0, 0, 0), 0.0f, 0.0f, 0.3f, 1.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_s_start(
        move, config, make_pose(0, 0, 0), 1.0f, 0.0f, 0.0f, 1.0f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_s_start(
        move, config, make_pose(0, 0, 0), 1.0f, 0.0f, 0.3f, 0.0f));
}

// Confirms update before start reports invalid state.
void test_update_before_start_is_invalid_state() {
    MoveS move = {};
    MoveSOutput output = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, move_s_update(
        move, make_pose(0, 0, 0), 0.02f, output));
}

// Confirms heading=0 drives straight along body-frame vx with no vy/omega.
void test_zero_heading_drives_along_vx() {
    const MoveSConfig config = make_config();
    MoveS move = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_s_start(
        move, config, make_pose(0, 0, 0), 1.0f, 0.0f, 0.3f, 1.0f));

    MoveSOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_s_update(move, make_pose(0, 0, 0), 0.02f, output));

    TEST_ASSERT_TRUE(output.requested_velocity.vx > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, output.requested_velocity.vy);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, output.requested_velocity.omega);
    TEST_ASSERT_EQUAL(static_cast<int>(MOVE_S_RUNNING), static_cast<int>(output.status));
}

// Confirms a strafe heading (pi/2) drives along body-frame vy instead.
void test_strafe_heading_drives_along_vy() {
    const MoveSConfig config = make_config();
    MoveS move = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_s_start(
        move, config, make_pose(0, 0, 0), 1.0f, kPi / 2.0f, 0.3f, 1.0f));

    MoveSOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_s_update(move, make_pose(0, 0, 0), 0.02f, output));

    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, output.requested_velocity.vx);
    TEST_ASSERT_TRUE(output.requested_velocity.vy > 0.0f);
}

// Confirms progress is measured along the direction captured at start, not
// re-derived from the current heading each cycle -- a heading reading that
// drifts during the move (calibration error) must not silently change what
// "remaining distance" means, since that would mask exactly the error §3's
// calibration trials are meant to measure.
void test_progress_ignores_current_heading_drift() {
    const MoveSConfig config = make_config();
    MoveS move = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_s_start(
        move, config, make_pose(0, 0, 0), 1.0f, 0.0f, 0.3f, 1.0f));

    MoveSOutput output_no_drift = {};
    MoveSOutput output_with_drift = {};
    MoveS move_copy = move;
    TEST_ASSERT_EQUAL(ESP_OK, move_s_update(move, make_pose(200.0f, 0, 0.0f), 0.02f, output_no_drift));
    TEST_ASSERT_EQUAL(ESP_OK, move_s_update(move_copy, make_pose(200.0f, 0, 0.3f), 0.02f, output_with_drift));

    TEST_ASSERT_FLOAT_WITHIN(kTolerance, output_no_drift.remaining_distance_m,
                              output_with_drift.remaining_distance_m);
}

// Confirms a full simulated run reaches completion within the configured
// tolerance, using a simple Euler integration of the commanded body
// velocity as a stand-in for a perfectly-tracking wheel-velocity PI. Jerk
// is left effectively unconstrained here so this test isolates MoveS's own
// progress-tracking/stopping-distance/completion logic; speed_profile's
// jerk-limiting behavior (which necessarily causes a little extra stopping
// distance of its own -- an accepted trade-off of the simplified ramp, see
// move_s.h) is already covered by test_speed_profile.
void test_simulated_run_reaches_target_distance() {
    MoveSConfig config = make_config(0.005f);
    config.speed_profile.max_jerk_mps3 = 1000.0f;
    MoveS move = {};
    const float distance_m = 1.0f;
    TEST_ASSERT_EQUAL(ESP_OK, move_s_start(
        move, config, make_pose(0, 0, 0), distance_m, 0.0f, 0.5f, 1.0f));

    DrivetrainPose pose = make_pose(0, 0, 0);
    constexpr float kDt = 0.01f;
    MoveSOutput output = {};
    int iterations = 0;
    constexpr int kMaxIterations = 5000;  // 50s simulated cap
    while (output.status != MOVE_S_COMPLETE && iterations < kMaxIterations) {
        TEST_ASSERT_EQUAL(ESP_OK, move_s_update(move, pose, kDt, output));
        pose.x_mm += output.requested_velocity.vx * kDt * 1000.0f;
        pose.y_mm += output.requested_velocity.vy * kDt * 1000.0f;
        ++iterations;
    }

    TEST_ASSERT_TRUE(iterations < kMaxIterations);
    TEST_ASSERT_EQUAL(static_cast<int>(MOVE_S_COMPLETE), static_cast<int>(output.status));
    TEST_ASSERT_TRUE(fabsf(output.remaining_distance_m) <= config.distance_tolerance_m + kTolerance);
    TEST_ASSERT_TRUE(fabsf(pose.y_mm) < kTolerance);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_rejects_invalid_config);
    RUN_TEST(test_start_rejects_invalid_parameters);
    RUN_TEST(test_update_before_start_is_invalid_state);
    RUN_TEST(test_zero_heading_drives_along_vx);
    RUN_TEST(test_strafe_heading_drives_along_vy);
    RUN_TEST(test_progress_ignores_current_heading_drift);
    RUN_TEST(test_simulated_run_reaches_target_distance);
    return UNITY_END();
}
