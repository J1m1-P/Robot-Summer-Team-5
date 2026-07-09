#pragma once 

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float x;
    float y;
    float turn; 
} DrivetrainBodyDuty;

typedef struct {
    float fl; 
    float fr;
    float bl; 
    float br;
} DrivetrainWheelDuty;

typedef struct {
    float wheel_angle_rad;
    float max_duty;
} DrivetrainKinematicsConfig;

esp_err_t drivetrain_kinematics_body_to_wheels(
    const DrivetrainKinematicsConfig *config, 
    const DrivetrainBodyDuty *body, 
    DrivetrainWheelDuty *wheels
);

#ifdef __cplusplus
}
#endif