#pragma once 

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#include "drivers/motor_driver.h"
#include "drivers/encoder_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

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
} DrivetrainConfig;

typedef struct {
    const DrivetrainConfig *config;

    MotorDriver motors[DRIVETRAIN_MOTOR_MAX];
    EncoderDriver encoders[DRIVETRAIN_MOTOR_MAX];

    bool initialized;
    bool enabled;

    float last_fl_duty;
    float last_fr_duty;
    float last_bl_duty;
    float last_br_duty;
} Drivetrain;

// Initialization
esp_err_t drivetrain_init(Drivetrain *drivetrain, const DrivetrainConfig *config);
esp_err_t drivetrain_enable(Drivetrain *drivetrain);
esp_err_t drivetrain_disable(Drivetrain *drivetrain);

// Motor Control
esp_err_t drivetrain_set_motor_duty(Drivetrain *drivetrain, DrivetrainMotorId motor_id, float duty);
esp_err_t drivetrain_set_all_motor_duties(Drivetrain *drivetrain, float fl_duty, float fr_duty, float bl_duty, float br_duty);
esp_err_t drivetrain_set_body_duty(Drivetrain *drivetrain, float x_duty, float y_duty, float turn_duty);

esp_err_t drivetrain_set_forward_duty(Drivetrain *drivetrain, float duty);
esp_err_t drivetrain_set_turn_duty(Drivetrain *drivetrain, float duty);
esp_err_t drivetrain_set_strafe_duty(Drivetrain *drivetrain, float duty);

esp_err_t drivetrain_coast(Drivetrain *drivetrain);
esp_err_t drivetrain_brake(Drivetrain *drivetrain);      //Active braking

// Encoder Control
esp_err_t drivetrain_encoder_update(Drivetrain *drivetrain);
int32_t drivetrain_get_encoder_count(Drivetrain *drivetrain, DrivetrainMotorId motor_id);
float drivetrain_get_encoder_velocity_mps(Drivetrain *drivetrain, DrivetrainMotorId motor_id);

// Implementation later
float drivetrain_get_x_velocity_mps(const Drivetrain *drivetrain);
float drivetrain_get_y_velocity_mps(const Drivetrain *drivetrain);
float drivetrain_get_turn_rate_radps(const Drivetrain *drivetrain);

#ifdef __cplusplus
}   
#endif
