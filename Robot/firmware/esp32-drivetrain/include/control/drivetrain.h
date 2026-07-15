#pragma once 

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "drivers/motor_driver.h"
#include "drivers/encoder_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRIVETRAIN_COMMAND_TIMEOUT_US 250000LL

typedef enum {
    DRIVETRAIN_MOTOR_FL = 0,
    DRIVETRAIN_MOTOR_FR,
    DRIVETRAIN_MOTOR_BL,
    DRIVETRAIN_MOTOR_BR,
    DRIVETRAIN_MOTOR_MAX
} DrivetrainMotorId;

typedef struct {
    const MotorDriverConfig *motor_configs[DRIVETRAIN_MOTOR_MAX];
    const EncoderDriverConfig *encoder_configs[DRIVETRAIN_MOTOR_MAX];

    float max_duty;
    float wheel_angle_rad;   // X-drive wheel force angle in radians

    uint8_t brk_pin;
} DrivetrainConfig;

typedef struct {
    const DrivetrainConfig *config;

    MotorDriver motors[DRIVETRAIN_MOTOR_MAX];
    EncoderDriver encoders[DRIVETRAIN_MOTOR_MAX];
    float last_duty[DRIVETRAIN_MOTOR_MAX];

    int64_t last_command_us;

    bool initialized;
    bool enabled;
    bool brake_engaged;
    bool command_timeout_active;
} Drivetrain;

// Initialization
// Zero-initialize the runtime object before its first init call:
// Drivetrain drivetrain = {0};
esp_err_t drivetrain_init(Drivetrain *drivetrain, const DrivetrainConfig *config);
// Enables every motor, then releases the brake. Rejects duplicate enable calls.
esp_err_t drivetrain_enable(Drivetrain *drivetrain);
// Stops all motors, engages the brake, and disables motor commands.
esp_err_t drivetrain_disable(Drivetrain *drivetrain);

// Motor Control
esp_err_t drivetrain_set_motor_duty(Drivetrain *drivetrain, DrivetrainMotorId motor_id, float duty);
esp_err_t drivetrain_set_all_motor_duty(Drivetrain *drivetrain, float fl_duty, float fr_duty, float bl_duty, float br_duty);
esp_err_t drivetrain_set_body_duty(Drivetrain *drivetrain, float x_duty, float y_duty, float turn_duty);

esp_err_t drivetrain_set_forward_duty(Drivetrain *drivetrain, float duty);
esp_err_t drivetrain_set_turn_duty(Drivetrain *drivetrain, float duty);
esp_err_t drivetrain_set_strafe_duty(Drivetrain *drivetrain, float duty);

// Explicitly sets PWM to zero and releases the brake for free movement.
esp_err_t drivetrain_coast(Drivetrain *drivetrain);
// Engages the brake and disables motor commands until drivetrain_enable().
esp_err_t drivetrain_brake(Drivetrain *drivetrain);

// Encoder Control
esp_err_t drivetrain_encoder_update(Drivetrain *drivetrain);
int32_t drivetrain_get_encoder_accumulated_count(const Drivetrain *drivetrain, DrivetrainMotorId motor_id);
float drivetrain_get_encoder_velocity_mps(const Drivetrain *drivetrain, DrivetrainMotorId motor_id);

// Watchdog: call once per control-loop iteration.
esp_err_t drivetrain_tick(Drivetrain *drivetrain, int64_t now_us);

#ifdef __cplusplus
}   
#endif
