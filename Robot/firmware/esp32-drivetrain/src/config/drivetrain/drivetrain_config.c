/* Defines the drivetrain's single hardware, geometry, control, and safety setup. */
#include "config/drivetrain/drivetrain_config.h"

#include "config/drivetrain/encoder_config.h"
#include "config/drivetrain/motor_config.h"
#include "config/drivetrain/move_calibration_config.h"
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

    .x_drive_kinematics = {
        .wheel_radius_m = 0.035f,
        .chassis_half_length_m = 0.100f,
        .chassis_half_width_m = 0.1365f,
        .wheel_angle_rad = DEG_TO_RAD(30.0f),
    },

    /* Locked in from tuning_main.cpp Duty-mode sweeps (kff/kff_offset) and
     * PI-mode step response (kp/ki), all four motors enabled. Measured
     * achievable wheel speed range at these gains: ~0.05 m/s (deadband
     * floor, below this feedforward can't reliably break static friction)
     * to ~0.54 m/s (output_max saturation ceiling) -- see
     * docs/TUNING_ROADMAP.md §5b/§5c. max_vx_mps/max_vy_mps below are not
     * yet reconciled against this: 1.0 m/s exceeds what a single wheel can
     * deliver even for pure-forward motion (~0.62 m/s at this ceiling via
     * the X-drive Jacobian's cos(30 deg) factor), so commanding anywhere
     * near max_vx_mps will saturate before reaching the requested speed. */
    .wheel_controller = {
        .kff = 1.2f,
        .kff_offset = 0.15f,
        .kp = 0.4f,
        .ki = 0.1f,
        .output_min = -0.95f,
        .output_max = 0.95f,
        .integral_min = -0.8f,
        .integral_max = 0.8f,
        /* No slew limiting needed on this hardware -- removed rather than
         * left at a small value. A finite-but-large rate (not 0) is how this
         * controller expresses "unlimited": duty_slew_per_s directly bounds
         * the per-cycle duty change (maximum_delta = duty_slew_per_s * dt),
         * so 0 would freeze duty at whatever it's already at, the opposite
         * of unlimited. 1000/s lets duty cross its entire +-0.8 output range
         * in under 2ms -- effectively instant against any control-loop dt
         * used here, matching the wheel-controller tests' own "very high
         * slew rate" convention for the unlimited case. This was capping
         * endpoint-settle's corrective pulses (~50ms) at ~0.1 duty when the
         * feedforward for their 0.08-0.12 m/s target wanted ~0.25-0.29 --
         * likely well under breakaway duty, which is why the pulse couldn't
         * reliably overcome static friction. */
        .duty_slew_per_s = 1000.0f,
        // Empirical startup assist for low-speed commands from rest.
        .breakaway_duty = 0.30f,
    },
    .max_duty = 0.95f,
    .max_vx_mps = 0.7f,
    .max_vy_mps = 0.5f,
    .max_omega_rad_s = 2.4f,
    .max_wheel_speed_mps = 0.7f,
    .max_control_dt_s = 0.05f,
    .command_timeout_us = 250000LL,
    .brake_pin = PIN_M_BRK,
    .move_calibration = &MOVE_CALIBRATION_CONFIG,
};
