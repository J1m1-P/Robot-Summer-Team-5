/* Tests MoveS (open-loop, dead-reckoned straight-line move) without ESP32 hardware. */
#include <unity.h>

#include <cmath>

#include "control/drivetrain/move_s.h"

namespace {

constexpr float kTolerance = 1.0e-4f;
constexpr float kPi = 3.14159265358979323846f;

MoveSConfig make_config(float distance_tolerance_m = 0.01f, float max_accel_mps2 = 0.5f) {
    MoveSConfig config = {};
    config.speed_profile.max_jerk_mps3 = 10.0f;
    config.distance_tolerance_m = distance_tolerance_m;
    config.max_accel_mps2 = max_accel_mps2;
    return config;
}

// Keeps test expressions concise while exercising the pointer-based C API.
esp_err_t move_s_start(
    MoveS &move,
    const MoveSConfig &config,
    float distance_m,
    float heading_rad,
    float max_speed_mps
) {
    return ::move_s_start(&move, &config, distance_m, heading_rad, max_speed_mps);
}

esp_err_t move_s_update(
    MoveS &move,
    float dt_s,
    MoveSOutput &output
) {
    return ::move_s_update(&move, dt_s, &output);
}

}  // namespace

void setUp() {}
void tearDown() {}

// Confirms a bad config (either distance tolerance or the acceleration
// ceiling) is rejected.
void test_rejects_invalid_config() {
    MoveS move = {};

    MoveSConfig bad_tolerance = make_config();
    bad_tolerance.distance_tolerance_m = -1.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_s_start(
        move, bad_tolerance, 1.0f, 0.0f, 0.3f));

    MoveSConfig bad_accel = make_config();
    bad_accel.max_accel_mps2 = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_s_start(
        move, bad_accel, 1.0f, 0.0f, 0.3f));
}

// Confirms start rejects non-positive distance/speed.
void test_start_rejects_invalid_parameters() {
    const MoveSConfig config = make_config();
    MoveS move = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_s_start(
        move, config, 0.0f, 0.0f, 0.3f));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_s_start(
        move, config, 1.0f, 0.0f, 0.0f));
}

// Confirms update before start reports invalid state.
void test_update_before_start_is_invalid_state() {
    MoveS move = {};
    MoveSOutput output = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, move_s_update(move, 0.02f, output));
}

// Confirms heading=0 drives straight along body-frame vx with no vy/omega.
void test_zero_heading_drives_along_vx() {
    const MoveSConfig config = make_config();
    MoveS move = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_s_start(move, config, 1.0f, 0.0f, 0.3f));

    MoveSOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_s_update(move, 0.02f, output));

    TEST_ASSERT_TRUE(output.requested_velocity.vx > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, output.requested_velocity.vy);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, output.requested_velocity.omega);
    TEST_ASSERT_EQUAL(static_cast<int>(MOVE_S_RUNNING), static_cast<int>(output.status));
}

// Confirms a strafe heading (pi/2) drives along body-frame vy instead.
void test_strafe_heading_drives_along_vy() {
    const MoveSConfig config = make_config();
    MoveS move = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_s_start(move, config, 1.0f, kPi / 2.0f, 0.3f));

    MoveSOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_s_update(move, 0.02f, output));

    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, output.requested_velocity.vx);
    TEST_ASSERT_TRUE(output.requested_velocity.vy > 0.0f);
}

// Confirms remaining distance shrinks by exactly commanded_speed * dt each
// cycle -- MoveS is genuinely open-loop (see move_s.h): there is no pose
// input at all anymore, so "remaining" can only ever come from this
// module's own self-integrated planned_progress_m. This replaces an older
// test that fed in a caller-supplied heading-drifted pose and confirmed
// progress ignored it -- that property is now structurally guaranteed
// (there is nothing left to ignore) rather than something to test.
void test_remaining_distance_tracks_self_integrated_progress() {
    const MoveSConfig config = make_config();
    MoveS move = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_s_start(move, config, 1.0f, 0.0f, 0.3f));

    MoveSOutput output_a = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_s_update(move, 0.02f, output_a));
    const float commanded_speed = hypotf(
        output_a.requested_velocity.vx, output_a.requested_velocity.vy);

    MoveSOutput output_b = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_s_update(move, 0.02f, output_b));

    TEST_ASSERT_FLOAT_WITHIN(
        kTolerance,
        output_a.remaining_distance_m - commanded_speed * 0.02f,
        output_b.remaining_distance_m);
}

// Confirms a full simulated run reaches completion within the configured
// tolerance. Unlike before, there is no pose to integrate here at all --
// MoveS's own planned_progress_m does all the tracking, so this test only
// needs to keep calling move_s_update() with a fixed dt. Jerk is left
// effectively unconstrained so this test isolates MoveS's own
// progress-tracking/stopping-distance/completion logic; speed_profile's
// jerk-limiting behavior (which necessarily causes a little extra stopping
// distance of its own -- an accepted trade-off of the simplified ramp, see
// move_s.h) is already covered by test_speed_profile.
void test_simulated_run_reaches_target_distance() {
    MoveSConfig config = make_config(0.005f, 1.0f);
    config.speed_profile.max_jerk_mps3 = 1000.0f;
    MoveS move = {};
    const float distance_m = 1.0f;
    TEST_ASSERT_EQUAL(ESP_OK, move_s_start(move, config, distance_m, 0.0f, 0.5f));

    constexpr float kDt = 0.01f;
    MoveSOutput output = {};
    int iterations = 0;
    constexpr int kMaxIterations = 5000;  // 50s simulated cap
    while (output.status != MOVE_S_COMPLETE && iterations < kMaxIterations) {
        TEST_ASSERT_EQUAL(ESP_OK, move_s_update(move, kDt, output));
        ++iterations;
    }

    TEST_ASSERT_TRUE(iterations < kMaxIterations);
    TEST_ASSERT_EQUAL(static_cast<int>(MOVE_S_COMPLETE), static_cast<int>(output.status));
    TEST_ASSERT_TRUE(fabsf(output.remaining_distance_m) <= config.distance_tolerance_m + kTolerance);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_rejects_invalid_config);
    RUN_TEST(test_start_rejects_invalid_parameters);
    RUN_TEST(test_update_before_start_is_invalid_state);
    RUN_TEST(test_zero_heading_drives_along_vx);
    RUN_TEST(test_strafe_heading_drives_along_vy);
    RUN_TEST(test_remaining_distance_tracks_self_integrated_progress);
    RUN_TEST(test_simulated_run_reaches_target_distance);
    return UNITY_END();
}
