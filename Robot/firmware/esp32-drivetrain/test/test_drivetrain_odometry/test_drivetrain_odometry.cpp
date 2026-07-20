/* Tests pure drivetrain pose integration without ESP32 hardware. */
#include <unity.h>

#include <cmath>

#include "control/drivetrain/odometry.h"

namespace {

constexpr float kTolerance = 1.0e-4f;

// Keeps test expressions concise while exercising the pointer-based C API.
esp_err_t drivetrain_odometry_update(
    DrivetrainOdometry &odometry,
    const DrivetrainOdometryDelta &delta,
    bool valid
) {
    return ::drivetrain_odometry_update(&odometry, &delta, valid);
}

// Keeps reset assertions concise while exercising the pointer-based C API.
void drivetrain_odometry_reset(DrivetrainOdometry &odometry) {
    ::drivetrain_odometry_reset(&odometry);
}

// Compares all fields of a pose against expected values.
void assert_pose(float x, float y, float heading, const DrivetrainPose &pose) {
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, x, pose.x_mm);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, y, pose.y_mm);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, heading, pose.heading_rad);
}

}  // namespace

void setUp() {}
void tearDown() {}

// Confirms a body-frame delta is accumulated at zero heading.
void test_integrates_delta_at_origin() {
    DrivetrainOdometry odometry = {};
    const DrivetrainOdometryDelta delta{100.0f, 25.0f, 0.2f};

    TEST_ASSERT_EQUAL(ESP_OK, drivetrain_odometry_update(odometry, delta, true));
    assert_pose(100.0f, 25.0f, 0.2f, odometry.pose);
}

// Confirms translation is rotated by the pose heading before accumulation.
void test_rotates_body_delta_into_world_frame() {
    DrivetrainOdometry odometry = {};
    odometry.pose.heading_rad = 3.14159265358979323846f / 2.0f;
    const DrivetrainOdometryDelta delta{100.0f, 0.0f, 0.0f};

    TEST_ASSERT_EQUAL(ESP_OK, drivetrain_odometry_update(odometry, delta, true));
    assert_pose(0.0f, -100.0f, 3.14159265358979323846f / 2.0f, odometry.pose);
}

// Confirms invalid sensor cycles hold pose and record a recoverable fault.
void test_invalid_cycle_holds_pose() {
    DrivetrainOdometry odometry = {};
    odometry.pose = {10.0f, 20.0f, 0.3f};

    TEST_ASSERT_EQUAL(ESP_OK, drivetrain_odometry_update(
        odometry, DrivetrainOdometryDelta{5.0f, 6.0f, 0.1f}, false));
    assert_pose(10.0f, 20.0f, 0.3f, odometry.pose);
    TEST_ASSERT_EQUAL(
        static_cast<int>(DRIVETRAIN_ODOMETRY_FAULT_INVALID_CYCLE),
        static_cast<int>(odometry.last_fault));
    TEST_ASSERT_TRUE(odometry.fault_latched);
}

// Confirms non-finite motion is rejected without changing the pose.
void test_rejects_non_finite_delta() {
    DrivetrainOdometry odometry = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, drivetrain_odometry_update(
        odometry, DrivetrainOdometryDelta{NAN, 0.0f, 0.0f}, true));
    assert_pose(0.0f, 0.0f, 0.0f, odometry.pose);
}

// Confirms reset clears both accumulated pose and fault history.
void test_reset_restores_defaults() {
    DrivetrainOdometry odometry = {};
    drivetrain_odometry_update(
        odometry, DrivetrainOdometryDelta{1.0f, 2.0f, 0.3f}, false);
    drivetrain_odometry_reset(odometry);

    assert_pose(0.0f, 0.0f, 0.0f, odometry.pose);
    TEST_ASSERT_EQUAL(
        static_cast<int>(DRIVETRAIN_ODOMETRY_FAULT_NONE),
        static_cast<int>(odometry.last_fault));
    TEST_ASSERT_FALSE(odometry.fault_latched);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_integrates_delta_at_origin);
    RUN_TEST(test_rotates_body_delta_into_world_frame);
    RUN_TEST(test_invalid_cycle_holds_pose);
    RUN_TEST(test_rejects_non_finite_delta);
    RUN_TEST(test_reset_restores_defaults);
    return UNITY_END();
}
