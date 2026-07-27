/* Tests the encoder-tick to world-frame odometry bridge without ESP32 hardware. */
#include <unity.h>

#include <cmath>

#include "control/drivetrain/drivetrain_odometry_source.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr uint32_t kCountsPerRevolution = 1000000;

// Shared geometry for every test: r=1, l=0.1, w=0.05, beta=30deg (matches
// test_x_drive_kinematics.cpp's convention).
DrivetrainOdometrySourceConfig make_config() {
    DrivetrainOdometrySourceConfig config = {};
    config.x_drive_kinematics.wheel_radius_m = 1.0f;
    config.x_drive_kinematics.chassis_half_length_m = 0.1f;
    config.x_drive_kinematics.chassis_half_width_m = 0.05f;
    config.x_drive_kinematics.wheel_angle_rad = 30.0f * kPi / 180.0f;
    config.counts_per_revolution_fl = kCountsPerRevolution;
    config.counts_per_revolution_fr = kCountsPerRevolution;
    config.counts_per_revolution_bl = kCountsPerRevolution;
    config.counts_per_revolution_br = kCountsPerRevolution;
    return config;
}

int32_t angle_to_counts(float angle_rad) {
    return static_cast<int32_t>(lroundf(angle_rad * kCountsPerRevolution / (2.0f * kPi)));
}

// Keeps test expressions concise while exercising the pointer-based C API.
esp_err_t drivetrain_odometry_source_update(
    DrivetrainOdometrySource &source,
    const DrivetrainOdometrySourceConfig &config,
    const DrivetrainWheelCounts &counts,
    DrivetrainOdometry &odometry
) {
    return ::drivetrain_odometry_source_update(&source, &config, &counts, &odometry);
}

void assert_pose(float x_mm, float y_mm, float heading_rad, const DrivetrainPose &pose) {
    constexpr float kQuantizationTolerance = 2.0f;
    constexpr float kAngleTolerance = 1.0e-3f;
    TEST_ASSERT_FLOAT_WITHIN(kQuantizationTolerance, x_mm, pose.x_mm);
    TEST_ASSERT_FLOAT_WITHIN(kQuantizationTolerance, y_mm, pose.y_mm);
    TEST_ASSERT_FLOAT_WITHIN(kAngleTolerance, heading_rad, pose.heading_rad);
}

}  // namespace

void setUp() {}
void tearDown() {}

// Confirms the first update after reset only captures a baseline: no motion
// is integrated (there is nothing to diff against yet), and it still
// reports ESP_OK per the "nothing to do yet is not an error" convention.
void test_first_update_only_captures_baseline() {
    DrivetrainOdometrySource source = {};
    const DrivetrainOdometrySourceConfig config = make_config();
    DrivetrainOdometry odometry = {};

    TEST_ASSERT_EQUAL(ESP_OK, drivetrain_odometry_source_update(
        source, config, DrivetrainWheelCounts{1000, 1000, 1000, 1000}, odometry));
    assert_pose(0.0f, 0.0f, 0.0f, odometry.pose);
}

// Confirms a known body-frame displacement, converted to per-wheel encoder
// ticks via the forward Jacobian, round-trips back through the odometry
// source into the same pose delta -- avoids hand-deriving wheel angles by
// reusing x_drive_kinematics as the source of truth for what ticks a given
// displacement should produce. At heading=0, body +x/+y align directly with
// world +x/+y: forward is +x and left is +y.
void test_integrates_known_displacement() {
    const DrivetrainOdometrySourceConfig config = make_config();

    const DrivetrainBodyVelocity displacement_m_rad{0.30f, -0.10f, 0.05f};
    XDriveWheelVelocity wheel_angle_delta_rad = {};
    TEST_ASSERT_EQUAL(ESP_OK, x_drive_kinematics_body_to_wheel_velocities(
        &config.x_drive_kinematics, &displacement_m_rad, &wheel_angle_delta_rad));

    const DrivetrainWheelCounts start_counts{0, 0, 0, 0};
    const DrivetrainWheelCounts end_counts{
        angle_to_counts(wheel_angle_delta_rad.fl),
        angle_to_counts(wheel_angle_delta_rad.fr),
        angle_to_counts(wheel_angle_delta_rad.bl),
        angle_to_counts(wheel_angle_delta_rad.br),
    };

    DrivetrainOdometrySource source = {};
    DrivetrainOdometry odometry = {};
    TEST_ASSERT_EQUAL(ESP_OK, drivetrain_odometry_source_update(
        source, config, start_counts, odometry));
    TEST_ASSERT_EQUAL(ESP_OK, drivetrain_odometry_source_update(
        source, config, end_counts, odometry));

    assert_pose(displacement_m_rad.vx * 1000.0f, displacement_m_rad.vy * 1000.0f,
                displacement_m_rad.omega, odometry.pose);
}

// Confirms successive cycles accumulate rather than replace the pose.
void test_accumulates_across_multiple_cycles() {
    const DrivetrainOdometrySourceConfig config = make_config();
    const int32_t step_counts = angle_to_counts(0.8660254f);  // theta = cos(30deg)

    DrivetrainOdometrySource source = {};
    DrivetrainOdometry odometry = {};
    TEST_ASSERT_EQUAL(ESP_OK, drivetrain_odometry_source_update(
        source, config, DrivetrainWheelCounts{0, 0, 0, 0}, odometry));

    // Pure-forward wheel deltas (all four wheels equal) advance x by
    // radius*theta/cos(30deg) = 1.0 m per step at this config's geometry.
    for (int step = 1; step <= 3; ++step) {
        const DrivetrainWheelCounts counts{
            step_counts * step, step_counts * step,
            step_counts * step, step_counts * step};
        TEST_ASSERT_EQUAL(ESP_OK, drivetrain_odometry_source_update(
            source, config, counts, odometry));
    }

    assert_pose(3000.0f, 0.0f, 0.0f, odometry.pose);
}

// Confirms an invalid config (zero encoder resolution) is rejected.
void test_rejects_invalid_config() {
    DrivetrainOdometrySourceConfig config = make_config();
    config.counts_per_revolution_fl = 0;

    DrivetrainOdometrySource source = {};
    DrivetrainOdometry odometry = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, drivetrain_odometry_source_update(
        source, config, DrivetrainWheelCounts{0, 0, 0, 0}, odometry));
}

// Confirms reset clears the tracked baseline so the next update re-captures
// one instead of diffing against stale counts.
void test_reset_forces_new_baseline() {
    const DrivetrainOdometrySourceConfig config = make_config();
    DrivetrainOdometrySource source = {};
    DrivetrainOdometry odometry = {};

    TEST_ASSERT_EQUAL(ESP_OK, drivetrain_odometry_source_update(
        source, config, DrivetrainWheelCounts{1000, 1000, 1000, 1000}, odometry));
    drivetrain_odometry_source_reset(&source);
    TEST_ASSERT_FALSE(source.has_previous_counts);

    TEST_ASSERT_EQUAL(ESP_OK, drivetrain_odometry_source_update(
        source, config, DrivetrainWheelCounts{5000, 5000, 5000, 5000}, odometry));
    assert_pose(0.0f, 0.0f, 0.0f, odometry.pose);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_first_update_only_captures_baseline);
    RUN_TEST(test_integrates_known_displacement);
    RUN_TEST(test_accumulates_across_multiple_cycles);
    RUN_TEST(test_rejects_invalid_config);
    RUN_TEST(test_reset_forces_new_baseline);
    return UNITY_END();
}
