/* Tests the shared off-tape motion PID behavior without ESP32 hardware. */
#include <unity.h>

#include <cmath>

#include "control/drivetrain/off_tape_motion.h"

namespace {

constexpr float kTolerance = 1.0e-4f;

OffTapeMotionConfig make_config(float kp, float dt_max_s = 0.05f) {
    OffTapeMotionConfig config = {};
    config.controller.proportional_gain = kp;
    config.controller.integral_gain = 0.0f;
    config.controller.derivative_gain = 0.0f;
    config.controller.integral_limit = 1.0f;
    config.controller.correction_min = -0.5f;
    config.controller.correction_max = 0.5f;
    config.controller_dt_max_s = dt_max_s;
    return config;
}

// Keeps test expressions concise while exercising the pointer-based C API.
esp_err_t off_tape_motion_init(OffTapeMotion &motion, const OffTapeMotionConfig &config) {
    return ::off_tape_motion_init(&motion, &config);
}

esp_err_t off_tape_motion_reset(OffTapeMotion &motion) {
    return ::off_tape_motion_reset(&motion);
}

esp_err_t off_tape_motion_update(
    OffTapeMotion &motion,
    const OffTapeMotionInput &input,
    float dt_s,
    OffTapeMotionOutput &output
) {
    return ::off_tape_motion_update(&motion, &input, dt_s, &output);
}

}  // namespace

void setUp() {}
void tearDown() {}

// Confirms a bad config is rejected and never becomes ready.
void test_rejects_invalid_config() {
    OffTapeMotion motion = {};
    OffTapeMotionConfig config = make_config(0.2f);
    config.controller_dt_max_s = -1.0f;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, off_tape_motion_init(motion, config));
}

// Confirms update refuses to run before init.
void test_update_before_init_is_invalid_state() {
    OffTapeMotion motion = {};
    OffTapeMotionOutput output = {};
    const OffTapeMotionConfig config = make_config(0.2f);

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, off_tape_motion_update(
        motion, OffTapeMotionInput{0.1f, 0.2f}, 0.02f, output));
    (void)config;
}

// Confirms travel velocity passes straight through to vx and the error is
// corrected laterally via the proportional gain, with omega left at zero.
void test_maps_error_to_lateral_correction() {
    OffTapeMotion motion = {};
    const OffTapeMotionConfig config = make_config(0.2f);
    TEST_ASSERT_EQUAL(ESP_OK, off_tape_motion_init(motion, config));

    OffTapeMotionOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, off_tape_motion_update(
        motion, OffTapeMotionInput{0.1f, 0.30f}, 0.02f, output));

    TEST_ASSERT_TRUE(output.motion_valid);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.30f, output.requested_velocity.vx);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.02f, output.requested_velocity.vy);
    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.0f, output.requested_velocity.omega);
}

// Confirms the correction saturates at correction_max for a large error.
void test_correction_saturates_at_bound() {
    OffTapeMotion motion = {};
    const OffTapeMotionConfig config = make_config(1.0f);
    TEST_ASSERT_EQUAL(ESP_OK, off_tape_motion_init(motion, config));

    OffTapeMotionOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, off_tape_motion_update(
        motion, OffTapeMotionInput{10.0f, 0.0f}, 0.02f, output));

    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.5f, output.requested_velocity.vy);
}

// Confirms a scheduling stall longer than controller_dt_max_s caps the
// integral contribution at controller_dt_max_s instead of the raw (huge)
// dt_s -- without the cap, this call's integral term would be error * 5.0
// instead of error * controller_dt_max_s.
void test_long_dt_caps_integral_contribution() {
    OffTapeMotion motion = {};
    OffTapeMotionConfig config = make_config(0.0f, 0.05f);
    config.controller.integral_gain = 1.0f;
    config.controller.integral_limit = 100.0f;
    config.controller.correction_min = -100.0f;
    config.controller.correction_max = 100.0f;
    TEST_ASSERT_EQUAL(ESP_OK, off_tape_motion_init(motion, config));

    OffTapeMotionOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, off_tape_motion_update(
        motion, OffTapeMotionInput{1.0f, 0.0f}, 5.0f, output));

    TEST_ASSERT_FLOAT_WITHIN(kTolerance, 0.05f, output.requested_velocity.vy);
}

// Confirms reset clears PID history without invalidating the stored config.
void test_reset_clears_history() {
    OffTapeMotion motion = {};
    const OffTapeMotionConfig config = make_config(0.0f, 0.5f);
    TEST_ASSERT_EQUAL(ESP_OK, off_tape_motion_init(motion, config));

    OffTapeMotionOutput output = {};
    TEST_ASSERT_EQUAL(ESP_OK, off_tape_motion_update(
        motion, OffTapeMotionInput{0.1f, 0.0f}, 0.02f, output));
    TEST_ASSERT_TRUE(motion.controller_state.has_previous_error);

    TEST_ASSERT_EQUAL(ESP_OK, off_tape_motion_reset(motion));
    TEST_ASSERT_FALSE(motion.controller_state.has_previous_error);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_rejects_invalid_config);
    RUN_TEST(test_update_before_init_is_invalid_state);
    RUN_TEST(test_maps_error_to_lateral_correction);
    RUN_TEST(test_correction_saturates_at_bound);
    RUN_TEST(test_long_dt_caps_integral_contribution);
    RUN_TEST(test_reset_clears_history);
    return UNITY_END();
}
