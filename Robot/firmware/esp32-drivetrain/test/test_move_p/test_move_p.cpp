#include <unity.h>
#include <cmath>
#include "control/drivetrain/move_p.h"

namespace {
MovePConfig config() {
    MovePConfig c = {};
    c.off_tape_motion.controller = {.proportional_gain=1.0f,.integral_gain=0,.derivative_gain=0,.integral_limit=1,.correction_min=-.5f,.correction_max=.5f};
    c.off_tape_motion.controller_dt_max_s=.05f;
    c.speed_profile.max_jerk_mps3=10.f;
    c.heading_controller = {.proportional_gain=1.f,.integral_gain=0,.derivative_gain=0,.integral_limit=1,.correction_min=-1.f,.correction_max=1.f};
    c.distance_tolerance_m=.01f; c.heading_tolerance_rad=.02f;
    c.max_accel_mps2=1.f; c.max_alpha_rad_s2=2.f; c.max_omega_rad_s=1.f;
    return c;
}
}
void setUp(){} void tearDown(){}
void test_move_p_starts_line_and_heading_trajectory() {
    MoveP m={}; const MovePConfig c=config();
    const MotionEstimate start={.x_m=0,.y_m=0,.heading_rad=0,.valid=true};
    TEST_ASSERT_EQUAL(ESP_OK,move_p_start(&m,&c,&start,1.f,0.f,1.f,0.3f));
    MovePOutput o={}; TEST_ASSERT_EQUAL(ESP_OK,move_p_update(&m,&start,.02f,&o));
    TEST_ASSERT_EQUAL(MOVE_P_TRANSLATE_AND_TURN,o.status);
    TEST_ASSERT_TRUE(o.requested_velocity.vx>0.f);
    TEST_ASSERT_TRUE(o.requested_velocity.omega>0.f);
}
void test_move_p_rejects_invalid_estimate() {
    MoveP m={}; const MovePConfig c=config(); const MotionEstimate bad={};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,move_p_start(&m,&c,&bad,1,0,0,.3f));
}
void test_move_p_completion_has_exact_zero_command() {
    MoveP m={}; const MovePConfig c=config();
    const MotionEstimate start={.x_m=0,.y_m=0,.heading_rad=0,.valid=true};
    TEST_ASSERT_EQUAL(ESP_OK,move_p_start(&m,&c,&start,1.f,0.f,0.f,0.3f));
    m.status = MOVE_P_SETTLE_HEADING;
    const MotionEstimate at_goal={.x_m=1,.y_m=0,.heading_rad=0,.valid=true};
    MovePOutput o={};
    TEST_ASSERT_EQUAL(ESP_OK,move_p_update(&m,&at_goal,.02f,&o));
    TEST_ASSERT_EQUAL(MOVE_P_COMPLETE,o.status);
    TEST_ASSERT_EQUAL_FLOAT(0.f,o.requested_velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.f,o.requested_velocity.vy);
    TEST_ASSERT_EQUAL_FLOAT(0.f,o.requested_velocity.omega);
}
void test_move_p_invalid_active_estimate_faults_with_zero_output() {
    MoveP m={}; const MovePConfig c=config();
    const MotionEstimate start={.x_m=0,.y_m=0,.heading_rad=0,.valid=true};
    TEST_ASSERT_EQUAL(ESP_OK,move_p_start(&m,&c,&start,1.f,0.f,0.f,0.3f));
    const MotionEstimate bad={}; MovePOutput o={};
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,move_p_update(&m,&bad,.02f,&o));
    TEST_ASSERT_EQUAL(MOVE_P_FAULT,o.status);
    TEST_ASSERT_EQUAL_FLOAT(0.f,o.requested_velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.f,o.requested_velocity.vy);
    TEST_ASSERT_EQUAL_FLOAT(0.f,o.requested_velocity.omega);
}
int main(int,char**){UNITY_BEGIN();RUN_TEST(test_move_p_starts_line_and_heading_trajectory);RUN_TEST(test_move_p_rejects_invalid_estimate);RUN_TEST(test_move_p_completion_has_exact_zero_command);RUN_TEST(test_move_p_invalid_active_estimate_faults_with_zero_output);return UNITY_END();}
