/* Defines the drivetrain's single hardware, geometry, control, and safety setup. */
#include "config/drivetrain/drivetrain_config.h"

#include "config/drivetrain/encoder_config.h"
#include "config/drivetrain/motor_config.h"
#include "config/pin_map.h"

#define DEG_TO_RAD(degrees) ((degrees) * 3.14159265358979323846f / 180.0f)

// Supplies every dependency and bound required by the velocity drivetrain.
const DrivetrainConfig DRIVETRAIN_CONFIG = {
    .motor_configs = {
        &FL_MOTOR_CONFIG,
        &FR_MOTOR_CONFIG,
        &BL_MOTOR_CONFIG,
        &BR_MOTOR_CONFIG,
    },
    .encoder_configs = {
        &FL_ENCODER_CONFIG,
        &FR_ENCODER_CONFIG,
        &BL_ENCODER_CONFIG,
        &BR_ENCODER_CONFIG,
    },
    .kinematics = {
        .wheel_radius_m = 0.035f,
        .chassis_half_length_m = 0.200f,
        .chassis_half_width_m = 0.273f,
        .wheel_angle_rad = DEG_TO_RAD(30.0f),
    },
    .wheel_pi = {
        .kff = 1.14f,
        .kff_offset = 0.069f,
        .kp = 0.4f,
        .ki = 0.2f,
        .output_min = -0.8f,
        .output_max = 0.8f,
        .integral_min = -0.8f,
        .integral_max = 0.8f,
        .duty_slew_per_s = 2.0f,
    },
    .max_duty = 0.8f,
    .max_vx_mps = 1.0f,
    .max_vy_mps = 1.0f,
    .max_omega_rad_s = 1.0f,
    .max_control_dt_s = 0.05f,
    .command_timeout_us = 250000LL,
    .brake_pin = PIN_M_BRK,
};
