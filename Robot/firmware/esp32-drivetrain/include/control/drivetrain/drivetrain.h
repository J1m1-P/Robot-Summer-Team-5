/* Declares the velocity-controlled drivetrain, lifecycle, and telemetry API. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "control/drivetrain/x_drive_kinematics.h"
#include "control/drivetrain/wheel_velocity_controller.h"
#include "drivers/encoder/encoder_driver.h"
#include "drivers/motor/motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

// Identifies wheels and fixes the internal array order as FL, FR, BL, BR.
typedef enum {
    DRIVETRAIN_MOTOR_FL = 0,
    DRIVETRAIN_MOTOR_FR,
    DRIVETRAIN_MOTOR_BL,
    DRIVETRAIN_MOTOR_BR,
    DRIVETRAIN_MOTOR_MAX,
} DrivetrainMotorId;

// Combines hardware, geometry, controller gains, motion limits, and safety.
typedef struct {
    const MotorDriverConfig *motor_configs[DRIVETRAIN_MOTOR_MAX];
    const EncoderDriverConfig *encoder_configs[DRIVETRAIN_MOTOR_MAX];
    XDriveKinematicsConfig x_drive_kinematics;
    WheelVelocityControllerConfig wheel_controller;
    float max_duty;
    float max_vx_mps;
    float max_vy_mps;
    float max_omega_rad_s;
    float max_control_dt_s;
    int64_t command_timeout_us;
    uint8_t brake_pin;
} DrivetrainConfig;

// Reports caller-relevant lifecycle, watchdog, and current body target state.
typedef struct {
    bool initialized;
    bool enabled;
    bool brake_engaged;
    bool command_timeout_active;
    DrivetrainBodyVelocity target_body;
} DrivetrainStatus;

// Groups the four hardware instances owned exclusively by the drivetrain.
typedef struct {
    MotorDriver motors[DRIVETRAIN_MOTOR_MAX];
    EncoderDriver encoders[DRIVETRAIN_MOTOR_MAX];
    WheelVelocityController wheel_controller[DRIVETRAIN_MOTOR_MAX];
} DrivetrainDevices;

// Groups closed-loop configuration, telemetry, and timing state.
typedef struct {
    WheelVelocityControllerConfig active_controller_config[DRIVETRAIN_MOTOR_MAX];
    float target_wheel_mps[DRIVETRAIN_MOTOR_MAX];
    float last_duty[DRIVETRAIN_MOTOR_MAX];
    int64_t last_command_us;
    int64_t last_update_us;
} DrivetrainControlState;

// Holds statically allocated drivetrain state; callers should use the API below.
typedef struct {
    const DrivetrainConfig *config;
    DrivetrainDevices devices;
    DrivetrainControlState control;
    DrivetrainStatus status;
} Drivetrain;

// Validates configuration and initializes the brake, motors, and encoders.
esp_err_t drivetrain_init(Drivetrain *drivetrain, const DrivetrainConfig *config);

// Enables every motor and releases the shared hardware brake.
esp_err_t drivetrain_enable(Drivetrain *drivetrain);

// Brakes the drivetrain and disables further motor output.
esp_err_t drivetrain_disable(Drivetrain *drivetrain);

// Stores a bounded body-frame velocity target and refreshes the watchdog.
esp_err_t drivetrain_set_body_velocity(
    Drivetrain *drivetrain,
    float vx_mps,
    float vy_mps,
    float omega_rad_s
);

// Sets a controlled zero-velocity target without disabling the drivetrain.
esp_err_t drivetrain_stop(Drivetrain *drivetrain);

// Runs one bounded-time encoder, kinematics, PI, and motor-output cycle.
esp_err_t drivetrain_update(Drivetrain *drivetrain, int64_t now_us);

// Copies the caller-facing lifecycle, watchdog, and target status.
esp_err_t drivetrain_get_status(
    const Drivetrain *drivetrain,
    DrivetrainStatus *status_out
);

// Engages the hardware brake, zeros PWM, and disables all motors.
esp_err_t drivetrain_brake(Drivetrain *drivetrain);

// Commands zero PWM while leaving the hardware brake released.
esp_err_t drivetrain_coast(Drivetrain *drivetrain);

// Returns one encoder's accumulated quadrature count, or zero if unavailable.
int32_t drivetrain_get_encoder_accumulated_count(
    const Drivetrain *drivetrain,
    DrivetrainMotorId motor_id
);

// Returns one wheel's latest measured linear velocity in meters per second.
float drivetrain_get_encoder_velocity_mps(
    const Drivetrain *drivetrain,
    DrivetrainMotorId motor_id
);

// Returns one wheel's current velocity target in meters per second.
float drivetrain_get_target_velocity_mps(
    const Drivetrain *drivetrain,
    DrivetrainMotorId motor_id
);

// Returns one wheel's most recently applied signed duty cycle.
float drivetrain_get_applied_duty(
    const Drivetrain *drivetrain,
    DrivetrainMotorId motor_id
);

// -----------------------------------------------------------------------------
// Real-time tuning only
// These APIs replace or inspect RAM-only wheel-controller configuration.
// Production motion code should use the compiled DrivetrainConfig defaults.
// -----------------------------------------------------------------------------

// Replaces the live controller configuration for all four wheels.
esp_err_t drivetrain_set_wheel_controller_config(
    Drivetrain *drivetrain,
    const WheelVelocityControllerConfig *config
);

// Copies the shared live configuration (the front-left wheel's current copy).
esp_err_t drivetrain_get_wheel_controller_config(
    const Drivetrain *drivetrain,
    WheelVelocityControllerConfig *config_out
);

// Replaces one wheel's live configuration without changing the other wheels.
esp_err_t drivetrain_set_motor_controller_config(
    Drivetrain *drivetrain,
    DrivetrainMotorId motor_id,
    const WheelVelocityControllerConfig *config
);

// Copies one wheel's live configuration for independent tuning.
esp_err_t drivetrain_get_motor_controller_config(
    const Drivetrain *drivetrain,
    DrivetrainMotorId motor_id,
    WheelVelocityControllerConfig *config_out
);

#ifdef __cplusplus
}
#endif
