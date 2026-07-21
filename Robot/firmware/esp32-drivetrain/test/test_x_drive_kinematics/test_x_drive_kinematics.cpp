// Test suite for the velocity Jacobian itself
// (control/drivetrain/x_drive_kinematics.h). This checks the formula's actual
// numeric output, its structural properties (linearity, radius scaling),
// and every error path.
//
// Run with: pio test -e native

#include <unity.h>

#include <cmath>

#include "control/drivetrain/x_drive_kinematics.h"

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kEpsilon = 1e-4f;

// Keeps test expressions concise while exercising the pointer-based C API.
esp_err_t x_drive_kinematics_body_to_wheel_velocities(
    const XDriveKinematicsConfig &config,
    const DrivetrainBodyVelocity &body,
    XDriveWheelVelocity &wheels
) {
    return ::x_drive_kinematics_body_to_wheel_velocities(
        &config, &body, &wheels);
}

XDriveKinematicsConfig make_config(float r, float l, float w, float beta_deg) {
    XDriveKinematicsConfig cfg = {};
    cfg.wheel_radius_m = r;
    cfg.chassis_half_length_m = l;
    cfg.chassis_half_width_m = w;
    cfg.wheel_angle_rad = beta_deg * kPi / 180.0f;
    return cfg;
}

DrivetrainBodyVelocity make_body(float vx, float vy, float omega) {
    DrivetrainBodyVelocity body = {};
    body.vx = vx;
    body.vy = vy;
    body.omega = omega;
    return body;
}

void assert_wheels_equal(const XDriveWheelVelocity &expected, const XDriveWheelVelocity &actual) {
    TEST_ASSERT_FLOAT_WITHIN(kEpsilon, expected.fl, actual.fl);
    TEST_ASSERT_FLOAT_WITHIN(kEpsilon, expected.fr, actual.fr);
    TEST_ASSERT_FLOAT_WITHIN(kEpsilon, expected.bl, actual.bl);
    TEST_ASSERT_FLOAT_WITHIN(kEpsilon, expected.br, actual.br);
}

}  // namespace

void setUp() {}
void tearDown() {}

// --- Exact numeric checks against hand-derived values ---
// Shared config for all of these: r=1, l=0.1, w=0.05, beta=30deg.
//   cos(30) = 0.8660254, sin(30) = 0.5
//   arm = l*sin(30) + w*cos(30) = 0.1*0.5 + 0.05*0.8660254 = 0.0933013

void test_zero_input_gives_zero_wheels() {
    const auto cfg = make_config(1.0f, 0.1f, 0.05f, 30.0f);
    const auto body = make_body(0.0f, 0.0f, 0.0f);
    XDriveWheelVelocity wheels = {};

    TEST_ASSERT_EQUAL(ESP_OK, x_drive_kinematics_body_to_wheel_velocities(cfg, body, wheels));

    XDriveWheelVelocity expected = {};
    assert_wheels_equal(expected, wheels); // all default-initialized to 0
}

void test_pure_forward_exact_values() {
    const auto cfg = make_config(1.0f, 0.1f, 0.05f, 30.0f);
    const auto body = make_body(2.0f, 0.0f, 0.0f);
    XDriveWheelVelocity wheels = {};

    TEST_ASSERT_EQUAL(ESP_OK, x_drive_kinematics_body_to_wheel_velocities(cfg, body, wheels));

    // cos(30)*2 = 1.7320508 for all four wheels (forward has no strafe/turn coupling)
    XDriveWheelVelocity expected = {};
    expected.fl = expected.fr = expected.bl = expected.br = 1.7320508f;
    assert_wheels_equal(expected, wheels);
}

void test_pure_strafe_exact_values() {
    const auto cfg = make_config(1.0f, 0.1f, 0.05f, 30.0f);
    const auto body = make_body(0.0f, 1.0f, 0.0f);
    XDriveWheelVelocity wheels = {};

    TEST_ASSERT_EQUAL(ESP_OK, x_drive_kinematics_body_to_wheel_velocities(cfg, body, wheels));

    // sin(30)*1 = 0.5, sign pattern +,-,-,+
    XDriveWheelVelocity expected = {};
    expected.fl = 0.5f;
    expected.fr = -0.5f;
    expected.bl = -0.5f;
    expected.br = 0.5f;
    assert_wheels_equal(expected, wheels);
}

void test_pure_turn_exact_values() {
    const auto cfg = make_config(1.0f, 0.1f, 0.05f, 30.0f);
    const auto body = make_body(0.0f, 0.0f, 1.0f);
    XDriveWheelVelocity wheels = {};

    TEST_ASSERT_EQUAL(ESP_OK, x_drive_kinematics_body_to_wheel_velocities(cfg, body, wheels));

    // arm*1 = 0.0933013, sign pattern -,+,-,+
    XDriveWheelVelocity expected = {};
    expected.fl = -0.0933013f;
    expected.fr = 0.0933013f;
    expected.bl = -0.0933013f;
    expected.br = 0.0933013f;
    assert_wheels_equal(expected, wheels);
}

void test_combined_motion_exact_values() {
    const auto cfg = make_config(1.0f, 0.1f, 0.05f, 30.0f);
    const auto body = make_body(1.0f, 0.5f, 0.2f);
    XDriveWheelVelocity wheels = {};

    TEST_ASSERT_EQUAL(ESP_OK, x_drive_kinematics_body_to_wheel_velocities(cfg, body, wheels));

    // cb*vx=0.8660254, sb*vy=0.25, arm*omega=0.01866025
    XDriveWheelVelocity expected = {};
    expected.fl = 0.8660254f + 0.25f - 0.01866025f;
    expected.fr = 0.8660254f - 0.25f + 0.01866025f;
    expected.bl = 0.8660254f - 0.25f - 0.01866025f;
    expected.br = 0.8660254f + 0.25f + 0.01866025f;
    assert_wheels_equal(expected, wheels);
}

// --- Structural invariants (don't depend on hand-picked constants) ---

void test_zero_moment_arm_removes_turn_coupling() {
    // l = w = 0 collapses arm to 0, so omega should contribute nothing to
    // any wheel, regardless of wheel_angle_rad or omega's magnitude.
    const auto cfg = make_config(1.0f, 0.0f, 0.0f, 30.0f);
    const auto body = make_body(0.0f, 0.0f, 5.0f);
    XDriveWheelVelocity wheels = {};

    TEST_ASSERT_EQUAL(ESP_OK, x_drive_kinematics_body_to_wheel_velocities(cfg, body, wheels));

    XDriveWheelVelocity expected = {}; // all zero
    assert_wheels_equal(expected, wheels);
}

void test_doubling_wheel_radius_halves_output() {
    const auto body = make_body(1.3f, -0.7f, 0.4f);

    const auto cfg_r1 = make_config(1.0f, 0.1f, 0.05f, 30.0f);
    const auto cfg_r2 = make_config(2.0f, 0.1f, 0.05f, 30.0f);

    XDriveWheelVelocity wheels_r1 = {};
    XDriveWheelVelocity wheels_r2 = {};
    TEST_ASSERT_EQUAL(ESP_OK, x_drive_kinematics_body_to_wheel_velocities(cfg_r1, body, wheels_r1));
    TEST_ASSERT_EQUAL(ESP_OK, x_drive_kinematics_body_to_wheel_velocities(cfg_r2, body, wheels_r2));

    XDriveWheelVelocity expected = {};
    expected.fl = wheels_r1.fl / 2.0f;
    expected.fr = wheels_r1.fr / 2.0f;
    expected.bl = wheels_r1.bl / 2.0f;
    expected.br = wheels_r1.br / 2.0f;
    assert_wheels_equal(expected, wheels_r2);
}

void test_linearity_superposition() {
    // The Jacobian is linear, so wheels(a) + wheels(b) == wheels(a + b)
    // for any two body-velocity inputs, on the same config.
    const auto cfg = make_config(0.7f, 0.12f, 0.09f, 30.0f);
    const auto body_a = make_body(0.6f, -0.3f, 0.9f);
    const auto body_b = make_body(-1.1f, 0.4f, -0.2f);
    const auto body_sum = make_body(body_a.vx + body_b.vx, body_a.vy + body_b.vy, body_a.omega + body_b.omega);

    XDriveWheelVelocity wheels_a = {};
    XDriveWheelVelocity wheels_b = {};
    XDriveWheelVelocity wheels_sum = {};
    TEST_ASSERT_EQUAL(ESP_OK, x_drive_kinematics_body_to_wheel_velocities(cfg, body_a, wheels_a));
    TEST_ASSERT_EQUAL(ESP_OK, x_drive_kinematics_body_to_wheel_velocities(cfg, body_b, wheels_b));
    TEST_ASSERT_EQUAL(ESP_OK, x_drive_kinematics_body_to_wheel_velocities(cfg, body_sum, wheels_sum));

    XDriveWheelVelocity expected = {};
    expected.fl = wheels_a.fl + wheels_b.fl;
    expected.fr = wheels_a.fr + wheels_b.fr;
    expected.bl = wheels_a.bl + wheels_b.bl;
    expected.br = wheels_a.br + wheels_b.br;
    assert_wheels_equal(expected, wheels_sum);
}

// --- Error paths ---

void test_rejects_non_finite_body_velocity() {
    const auto cfg = make_config(1.0f, 0.1f, 0.05f, 30.0f);
    XDriveWheelVelocity wheels = {};

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        x_drive_kinematics_body_to_wheel_velocities(cfg, make_body(NAN, 0.0f, 0.0f), wheels));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        x_drive_kinematics_body_to_wheel_velocities(cfg, make_body(0.0f, INFINITY, 0.0f), wheels));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        x_drive_kinematics_body_to_wheel_velocities(cfg, make_body(0.0f, 0.0f, -INFINITY), wheels));
}

void test_rejects_non_positive_wheel_radius() {
    const auto body = make_body(1.0f, 0.0f, 0.0f);
    XDriveWheelVelocity wheels = {};

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        x_drive_kinematics_body_to_wheel_velocities(make_config(0.0f, 0.1f, 0.05f, 30.0f), body, wheels));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        x_drive_kinematics_body_to_wheel_velocities(make_config(-1.0f, 0.1f, 0.05f, 30.0f), body, wheels));
}

void test_rejects_non_finite_wheel_radius() {
    auto cfg = make_config(1.0f, 0.1f, 0.05f, 30.0f);
    cfg.wheel_radius_m = NAN;
    const auto body = make_body(1.0f, 0.0f, 0.0f);
    XDriveWheelVelocity wheels = {};

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, x_drive_kinematics_body_to_wheel_velocities(cfg, body, wheels));
}

void test_rejects_non_finite_wheel_angle() {
    auto cfg = make_config(1.0f, 0.1f, 0.05f, 30.0f);
    cfg.wheel_angle_rad = NAN;
    const auto body = make_body(1.0f, 0.0f, 0.0f);
    XDriveWheelVelocity wheels = {};

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, x_drive_kinematics_body_to_wheel_velocities(cfg, body, wheels));
}

// Confirms the inverse Jacobian recovers a combined body command.
void test_round_trip_body_to_wheel_to_body() {
    const auto config = make_config(0.035f, 0.20f, 0.273f, 30.0f);
    const DrivetrainBodyVelocity expected{0.31f, -0.17f, 0.42f};
    XDriveWheelVelocity wheels{};
    DrivetrainBodyVelocity actual{};
    TEST_ASSERT_EQUAL(ESP_OK, x_drive_kinematics_body_to_wheel_velocities(
        &config, &expected, &wheels));
    TEST_ASSERT_EQUAL(ESP_OK, x_drive_kinematics_wheel_to_body_velocities(
        &config, &wheels, &actual));
    TEST_ASSERT_FLOAT_WITHIN(kEpsilon, expected.vx, actual.vx);
    TEST_ASSERT_FLOAT_WITHIN(kEpsilon, expected.vy, actual.vy);
    TEST_ASSERT_FLOAT_WITHIN(kEpsilon, expected.omega, actual.omega);
}

// Rejects inverse transforms whose geometry cannot distinguish all axes.
void test_inverse_rejects_singular_geometry() {
    auto config = make_config(0.035f, 0.20f, 0.273f, 30.0f);
    const XDriveWheelVelocity wheels{};
    DrivetrainBodyVelocity body{};
    config.wheel_angle_rad = 0.0f;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        x_drive_kinematics_wheel_to_body_velocities(&config, &wheels, &body));
}

void test_rejects_invalid_chassis_dimensions() {
    const auto body = make_body(1.0f, 0.0f, 0.0f);
    XDriveWheelVelocity wheels = {};
    auto cfg = make_config(1.0f, -0.1f, 0.05f, 30.0f);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        x_drive_kinematics_body_to_wheel_velocities(cfg, body, wheels));

    cfg = make_config(1.0f, 0.1f, NAN, 30.0f);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        x_drive_kinematics_body_to_wheel_velocities(cfg, body, wheels));
}

int main(int argc, char **argv) {
    UNITY_BEGIN();

    RUN_TEST(test_zero_input_gives_zero_wheels);
    RUN_TEST(test_pure_forward_exact_values);
    RUN_TEST(test_pure_strafe_exact_values);
    RUN_TEST(test_pure_turn_exact_values);
    RUN_TEST(test_combined_motion_exact_values);
    RUN_TEST(test_round_trip_body_to_wheel_to_body);
    RUN_TEST(test_inverse_rejects_singular_geometry);

    RUN_TEST(test_zero_moment_arm_removes_turn_coupling);
    RUN_TEST(test_doubling_wheel_radius_halves_output);
    RUN_TEST(test_linearity_superposition);

    RUN_TEST(test_rejects_non_finite_body_velocity);
    RUN_TEST(test_rejects_non_positive_wheel_radius);
    RUN_TEST(test_rejects_non_finite_wheel_radius);
    RUN_TEST(test_rejects_non_finite_wheel_angle);
    RUN_TEST(test_rejects_invalid_chassis_dimensions);

    return UNITY_END();
}
