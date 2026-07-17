// Sign-parity check between the existing open-loop duty mixer
// (control/drivetrain_kinematics.h) and the new velocity Jacobian
// (control/drivetrain_velocity_kinematics.h): for a pure single-axis
// command (forward-only, strafe-only, or turn-only), both models should
// spin every wheel in the same direction.
//
// Deliberately NOT tested: combined motions (e.g. forward + strafe
// together). The old model divides by sin/cos of the wheel angle while
// the new one multiplies, so the two weight the strafe axis relative to
// forward/turn differently -- a combined-motion case can legitimately
// disagree between them without either being "wrong" for its own model.
// Isolating one axis at a time cancels that discrepancy out.

#include <unity.h>

extern "C" {
#include "control/drivetrain_kinematics.h"
}
#include "control/drivetrain_velocity_kinematics.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kWheelAngleRad = 30.0f * kPi / 180.0f;

DrivetrainKinematicsConfig make_old_config() {
    DrivetrainKinematicsConfig cfg{};
    cfg.wheel_angle_rad = kWheelAngleRad;
    cfg.max_duty = 1.0f;
    return cfg;
}

DrivetrainVelocityKinematicsConfig make_new_config() {
    DrivetrainVelocityKinematicsConfig cfg;
    cfg.wheel_radius_m = 1.0f;
    cfg.chassis_half_length_m = 0.1f;
    cfg.chassis_half_width_m = 0.1f;
    cfg.wheel_angle_rad = kWheelAngleRad;
    return cfg;
}

// -1, 0, or +1; small values count as zero so exact-zero terms in one
// model don't get treated as a sign mismatch against float noise in the
// other.
int sign_of(float v) {
    constexpr float kEpsilon = 1e-4f;
    if (v > kEpsilon) return 1;
    if (v < -kEpsilon) return -1;
    return 0;
}

void assert_same_sign_pattern(const DrivetrainWheelDuty &old_wheels, const DrivetrainWheelVelocity &new_wheels) {
    TEST_ASSERT_EQUAL_INT(sign_of(old_wheels.fl), sign_of(new_wheels.fl));
    TEST_ASSERT_EQUAL_INT(sign_of(old_wheels.fr), sign_of(new_wheels.fr));
    TEST_ASSERT_EQUAL_INT(sign_of(old_wheels.bl), sign_of(new_wheels.bl));
    TEST_ASSERT_EQUAL_INT(sign_of(old_wheels.br), sign_of(new_wheels.br));
}

void run_parity_case(float old_x, float old_y, float old_turn, float new_vx, float new_vy, float new_omega) {
    const DrivetrainKinematicsConfig old_cfg = make_old_config();
    const DrivetrainVelocityKinematicsConfig new_cfg = make_new_config();

    DrivetrainBodyDuty old_body{};
    old_body.x = old_x;
    old_body.y = old_y;
    old_body.turn = old_turn;

    DrivetrainBodyVelocity new_body;
    new_body.vx = new_vx;
    new_body.vy = new_vy;
    new_body.omega = new_omega;

    DrivetrainWheelDuty old_wheels{};
    DrivetrainWheelVelocity new_wheels;

    TEST_ASSERT_EQUAL(ESP_OK, drivetrain_kinematics_body_to_wheels(&old_cfg, &old_body, &old_wheels));
    TEST_ASSERT_EQUAL(ESP_OK, drivetrain_kinematics_body_to_wheel_velocities(new_cfg, new_body, new_wheels));

    assert_same_sign_pattern(old_wheels, new_wheels);
}

}  // namespace

void setUp() {}
void tearDown() {}

void test_pure_forward_sign_parity() {
    run_parity_case(/*old x,y,turn=*/ 0.0f, 1.0f, 0.0f, /*new vx,vy,omega=*/ 1.0f, 0.0f, 0.0f);
}

void test_pure_strafe_sign_parity() {
    run_parity_case(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f);
}

void test_pure_turn_sign_parity() {
    run_parity_case(0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_pure_forward_sign_parity);
    RUN_TEST(test_pure_strafe_sign_parity);
    RUN_TEST(test_pure_turn_sign_parity);
    return UNITY_END();
}
