#include <unity.h>

#include "control/drivetrain/motion_estimate_adapter.h"

void setUp() {}
void tearDown() {}

void test_converts_millimetres_to_valid_world_metres() {
    const DrivetrainPose pose = {.x_mm = 1250.0f, .y_mm = -500.0f, .heading_rad = 0.75f};
    MotionEstimate estimate = {};
    TEST_ASSERT_EQUAL(ESP_OK, motion_estimate_from_drivetrain_pose(&pose, true, &estimate));
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, 1.25f, estimate.x_m);
    TEST_ASSERT_FLOAT_WITHIN(1.0e-6f, -0.5f, estimate.y_m);
    TEST_ASSERT_TRUE(estimate.valid);
}

void test_unhealthy_estimate_is_not_usable() {
    const DrivetrainPose pose = {};
    MotionEstimate estimate = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
        motion_estimate_from_drivetrain_pose(&pose, false, &estimate));
    TEST_ASSERT_FALSE(estimate.valid);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_converts_millimetres_to_valid_world_metres);
    RUN_TEST(test_unhealthy_estimate_is_not_usable);
    return UNITY_END();
}
