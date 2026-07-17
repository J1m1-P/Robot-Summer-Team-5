#include "config/drivetrain_velocity_kinematics_config.h"

namespace {

constexpr float deg_to_rad(float deg) {
    return deg * 3.14159265358979323846f / 180.0f;
}

// kWheelAngleDeg matches the duty-based model's WHEEL_ANGLE in
// config/drivetrain_config.c (30 deg, same X-drive geometry) -- keep them
// in sync if either changes.
constexpr float kWheelAngleDeg = 30.0f;
constexpr float kWheelRadiusM = 0.070f;         // r -- TODO: verify against ENCODER_WHEEL_DIAMETER_M
                                                 // in config/encoder_config.c (also 0.070) -- one of
                                                 // these is labeled diameter, the other radius; they
                                                 // shouldn't be the same number. Currently doesn't
                                                 // affect behavior (r cancels out of the kinematics'
                                                 // rad/s output once drive_main.cpp converts back to
                                                 // m/s via the same r), but worth fixing for clarity.

// Measured directly on the chassis (were 0.02/0.0273 placeholders before --
// off by exactly 10x, almost certainly a decimal-place slip entering
// 200mm/273mm as meters). This 10x undercounted every turn command's real
// wheel-speed target: at omega=1 rad/s, arm = l*sin(beta)+w*cos(beta) is
// ~0.336 here vs ~0.0336 with the old placeholders, so the real
// feedforward duty jump at turn start (kff*target+kff_offset, ~0.45 at
// kff=1.14) is about 10x what earlier analysis assumed -- a real,
// substantial instantaneous 4-motor duty step, not the negligible one the
// placeholder geometry suggested. Directly relevant to the suspected
// boost-converter incident.
constexpr float kChassisHalfLengthM = 0.200f;  // l, center to axle, front-back (measured, was 200mm)
constexpr float kChassisHalfWidthM = 0.273f;   // w, center to wheel, left-right (measured, was 273mm)

}  // namespace

const DrivetrainVelocityKinematicsConfig DRIVETRAIN_VELOCITY_KINEMATICS_CONFIG = {
    kWheelRadiusM,
    kChassisHalfLengthM,
    kChassisHalfWidthM,
    deg_to_rad(kWheelAngleDeg),
};
