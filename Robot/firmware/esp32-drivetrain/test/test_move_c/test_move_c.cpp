#include <unity.h>
#include <cmath>

#include "control/drivetrain/move_c.h"

namespace {
MoveCConfig config() {
    MoveCConfig c = {};
    c.off_tape_motion.controller = {
        .proportional_gain = 1.0f, .integral_gain = 0.0f,
        .derivative_gain = 0.0f, .integral_limit = 1.0f,
        .correction_min = -0.5f, .correction_max = 0.5f};
    c.off_tape_motion.controller_dt_max_s = 0.05f;
    c.speed_profile.max_jerk_mps3 = 10.0f;
    c.heading_controller = {
        .proportional_gain = 1.0f, .integral_gain = 0.0f,
        .derivative_gain = 0.0f, .integral_limit = 1.0f,
        .correction_min = -1.0f, .correction_max = 1.0f};
    c.arc_length_tolerance_m = 0.02f;
    c.radial_tolerance_m = 0.02f;
    c.heading_tolerance_rad = 0.02f;
    c.max_accel_mps2 = 1.0f;
    c.max_alpha_rad_s2 = 2.0f;
    c.max_omega_rad_s = 2.0f;
    return c;
}
}

void setUp() {}
void tearDown() {}

void test_move_c_starts_with_tangent_translation_and_turn() {
    MoveC m = {};
    const MoveCConfig c = config();
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f,
                                  .heading_rad = 0.0f, .valid = true};
    TEST_ASSERT_EQUAL(ESP_OK, move_c_start(&m, &c, &start, 1.0f,
                                           1.5707963f, 0.3f));
    MoveCOutput out = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_c_update(&m, &start, 0.02f, &out));
    TEST_ASSERT_EQUAL(MOVE_C_RUNNING, out.status);
    TEST_ASSERT_TRUE(out.requested_velocity.vx > 0.0f);
    TEST_ASSERT_TRUE(out.requested_velocity.omega > 0.0f);
}

void test_move_c_radial_error_commands_toward_arc() {
    MoveC m = {};
    const MoveCConfig c = config();
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f,
                                  .heading_rad = 0.0f, .valid = true};
    TEST_ASSERT_EQUAL(ESP_OK, move_c_start(&m, &c, &start, 1.0f,
                                           1.5707963f, 0.3f));
    // Positive CCW arc: this point is outside the circle, so correction must
    // point inward (negative body vy at the start; body +y is left).
    const MotionEstimate outside = {.x_m = 0.0f, .y_m = -0.1f,
                                    .heading_rad = 0.0f, .valid = true};
    MoveCOutput out = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_c_update(&m, &outside, 0.02f, &out));
    TEST_ASSERT_TRUE(out.radial_error_m > 0.0f);
    TEST_ASSERT_TRUE(out.requested_velocity.vy < 0.0f);
}

void test_move_c_clockwise_radial_correction_reverses_sign() {
    MoveC m = {};
    const MoveCConfig c = config();
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f,
                                  .heading_rad = 0.0f, .valid = true};
    TEST_ASSERT_EQUAL(ESP_OK, move_c_start(&m, &c, &start, 1.0f,
                                           -1.5707963f, 0.3f));
    // For a clockwise arc the center is on world -y/right. This point is
    // outside the circle on world +y/left, so correction is positive body vy.
    const MotionEstimate outside = {.x_m = 0.0f, .y_m = 0.1f,
                                    .heading_rad = 0.0f, .valid = true};
    MoveCOutput out = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_c_update(&m, &outside, 0.02f, &out));
    TEST_ASSERT_TRUE(out.radial_error_m > 0.0f);
    TEST_ASSERT_TRUE(out.requested_velocity.vy > 0.0f);
}

void test_move_c_completion_has_exact_zero_command() {
    MoveC m = {};
    const MoveCConfig c = config();
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f,
                                  .heading_rad = 0.0f, .valid = true};
    TEST_ASSERT_EQUAL(ESP_OK, move_c_start(&m, &c, &start, 1.0f,
                                           1.5707963f, 0.3f));
    // Endpoint of a quarter-circle centered at (0,1).
    const MotionEstimate endpoint = {.x_m = 1.0f, .y_m = 1.0f,
                                     .heading_rad = 1.5707963f, .valid = true};
    MoveCOutput out = {};
    TEST_ASSERT_EQUAL(ESP_OK, move_c_update(&m, &endpoint, 0.02f, &out));
    TEST_ASSERT_EQUAL(MOVE_C_COMPLETE, out.status);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.requested_velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.requested_velocity.vy);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.requested_velocity.omega);
}

void test_move_c_invalid_active_estimate_faults_with_zero_output() {
    MoveC m = {};
    const MoveCConfig c = config();
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f,
                                  .heading_rad = 0.0f, .valid = true};
    TEST_ASSERT_EQUAL(ESP_OK, move_c_start(&m, &c, &start, 1.0f,
                                           1.5707963f, 0.3f));
    const MotionEstimate bad = {};
    MoveCOutput out = {};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, move_c_update(&m, &bad, 0.02f, &out));
    TEST_ASSERT_EQUAL(MOVE_C_FAULT, out.status);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.requested_velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.requested_velocity.vy);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.requested_velocity.omega);
}

void test_move_c_stalled_arc_progress_faults_instead_of_hanging() {
    MoveC m = {};
    const MoveCConfig c = config();
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f,
                                  .heading_rad = 0.0f, .valid = true};
    TEST_ASSERT_EQUAL(ESP_OK, move_c_start(&m, &c, &start, 1.0f,
                                           1.5707963f, 0.3f));
    // A point on the same quarter-circle (center (0,1), radius 1) 0.05 m of
    // arc-length short of the endpoint -- simulates wheel slip that leaves
    // the robot permanently unable to close the remaining arc-length gap
    // (bigger than arc_length_tolerance_m) once its one-way profile stops.
    const MotionEstimate stalled = {.x_m = 0.99875026f, .y_m = 0.95002083f,
                                    .heading_rad = 1.5208f, .valid = true};
    MoveCOutput out = {};
    MoveCStatus status = MOVE_C_RUNNING;
    for (int i = 0; i < 500 && status == MOVE_C_RUNNING; ++i) {
        TEST_ASSERT_EQUAL(ESP_OK, move_c_update(&m, &stalled, 0.02f, &out));
        status = out.status;
    }
    TEST_ASSERT_EQUAL(MOVE_C_FAULT, status);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.requested_velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.requested_velocity.vy);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.requested_velocity.omega);
}

void test_move_c_endpoint_radial_residual_pulses_above_deadband() {
    MoveC m = {};
    const MoveCConfig c = config();
    const MotionEstimate start = {.x_m = 0.0f, .y_m = 0.0f,
                                  .heading_rad = 0.0f, .valid = true};
    TEST_ASSERT_EQUAL(ESP_OK, move_c_start(&m, &c, &start, 1.0f,
                                           1.5707963f, 0.3f));
    // Endpoint of the quarter-circle (center (0,1)), but 0.03 m radially
    // outside the target circle -- past this config's radial_tolerance_m
    // (0.02f). A plain continuous PID (proportional_gain = 1.0) would
    // command only 0.03 m/s here, well under the ~0.05 m/s characterized
    // wheel-velocity floor, and would never actually move the robot.
    const MotionEstimate stalled = {.x_m = 1.03f, .y_m = 1.0f,
                                    .heading_rad = 1.5707963f, .valid = true};
    MoveCOutput out = {};

    // Pulse windows (0.08 s) survive the first four 0.02 s steps and command
    // a body speed clearly above the deadband, not the tiny continuous value.
    for (int i = 0; i < 4; ++i) {
        TEST_ASSERT_EQUAL(ESP_OK, move_c_update(&m, &stalled, 0.02f, &out));
        TEST_ASSERT_EQUAL(MOVE_C_RUNNING, out.status);
        const float speed = hypotf(out.requested_velocity.vx, out.requested_velocity.vy);
        TEST_ASSERT_TRUE(speed >= 0.10f - 1.0e-4f);
        TEST_ASSERT_TRUE(speed <= 0.20f + 1.0e-4f);
    }
    // The pause window (0.10 s) then commands exact zero, not a residual
    // sub-deadband trickle.
    TEST_ASSERT_EQUAL(ESP_OK, move_c_update(&m, &stalled, 0.02f, &out));
    TEST_ASSERT_EQUAL(MOVE_C_RUNNING, out.status);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.requested_velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.requested_velocity.vy);
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_move_c_starts_with_tangent_translation_and_turn);
    RUN_TEST(test_move_c_radial_error_commands_toward_arc);
    RUN_TEST(test_move_c_clockwise_radial_correction_reverses_sign);
    RUN_TEST(test_move_c_completion_has_exact_zero_command);
    RUN_TEST(test_move_c_invalid_active_estimate_faults_with_zero_output);
    RUN_TEST(test_move_c_stalled_arc_progress_faults_instead_of_hanging);
    RUN_TEST(test_move_c_endpoint_radial_residual_pulses_above_deadband);
    return UNITY_END();
}
