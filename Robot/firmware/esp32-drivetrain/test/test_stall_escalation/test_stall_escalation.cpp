// Test suite for control/motion/stall_escalation.h.
//
// Run with: pio test -e native

#include <unity.h>

#include "control/motion/stall_escalation.h"

void setUp() {}
void tearDown() {}

void test_no_escalation_while_making_progress() {
    StallEscalation state = {};
    for (int i = 0; i < 5; ++i) {
        const float scale = stall_escalation_update(&state, 0.1f, 0.002f, 0.005f);
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, scale);
    }
}

void test_no_escalation_when_commanded_speed_is_near_zero() {
    StallEscalation state = {};
    for (int i = 0; i < 100; ++i) {
        const float scale = stall_escalation_update(&state, 0.0f, 0.0f, 0.005f);
        TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, scale);
    }
}

void test_escalates_after_grace_period_when_stalled() {
    StallEscalation state = {};
    float scale = 1.0f;
    // 0.6s of stall at dt=0.005s is well past the 0.5s grace period.
    for (int i = 0; i < 120; ++i) {
        scale = stall_escalation_update(&state, 0.1f, 0.0f, 0.005f);
    }
    TEST_ASSERT_TRUE(scale > 1.0f);
}

void test_resets_to_baseline_once_progress_resumes() {
    StallEscalation state = {};
    for (int i = 0; i < 120; ++i) {
        stall_escalation_update(&state, 0.1f, 0.0f, 0.005f);
    }
    const float scale = stall_escalation_update(&state, 0.1f, 0.01f, 0.005f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, scale);
}

void test_escalation_is_bounded() {
    StallEscalation state = {};
    float scale = 1.0f;
    for (int i = 0; i < 2000; ++i) {
        scale = stall_escalation_update(&state, 0.1f, 0.0f, 0.005f);
    }
    TEST_ASSERT_TRUE(scale <= 3.0f);
}

void test_apply_scale_is_noop_under_limits() {
    float vx = 0.1f, vy = 0.05f;
    stall_escalation_apply_scale(&vx, &vy, 2.0f, 0.7f, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f, vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.1f, vy);
}

// Regression test: max_vx_mps (0.7) and max_vy_mps (0.5) differ, so clamping
// each axis independently would change the vx:vy ratio -- exactly the
// direction distortion this vector-scaling approach exists to avoid.
void test_apply_scale_preserves_direction_when_one_axis_would_clip() {
    float vx = 0.3f, vy = 0.3f;
    const float original_ratio = vx / vy;
    stall_escalation_apply_scale(&vx, &vy, 3.0f, 0.7f, 0.5f);

    TEST_ASSERT_FLOAT_WITHIN(1e-4f, original_ratio, vx / vy);
    TEST_ASSERT_TRUE(vx <= 0.7f + 1e-4f);
    TEST_ASSERT_TRUE(vy <= 0.5f + 1e-4f);
}

void test_apply_scale_handles_zero_command() {
    float vx = 0.0f, vy = 0.0f;
    stall_escalation_apply_scale(&vx, &vy, 3.0f, 0.7f, 0.5f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, vy);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_no_escalation_while_making_progress);
    RUN_TEST(test_no_escalation_when_commanded_speed_is_near_zero);
    RUN_TEST(test_escalates_after_grace_period_when_stalled);
    RUN_TEST(test_resets_to_baseline_once_progress_resumes);
    RUN_TEST(test_escalation_is_bounded);
    RUN_TEST(test_apply_scale_is_noop_under_limits);
    RUN_TEST(test_apply_scale_preserves_direction_when_one_axis_would_clip);
    RUN_TEST(test_apply_scale_handles_zero_command);

    return UNITY_END();
}
