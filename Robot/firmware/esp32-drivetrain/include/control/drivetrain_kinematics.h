/* Declares conversion from body-axis duty commands to four X-drive wheel duties. */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Represents normalized translation and turning duty in the robot body frame.
typedef struct {
    float x;
    float y;
    float turn; 
} DrivetrainBodyDuty;

// Represents the four signed wheel duties produced by kinematic mixing.
typedef struct {
    float fl; 
    float fr;
    float bl; 
    float br;
} DrivetrainWheelDuty;

// Defines the X-drive wheel angle and maximum permitted duty magnitude.
typedef struct {
    float wheel_angle_rad;
    float max_duty;
} DrivetrainKinematicsConfig;

// Mixes a body command into wheel duties and scales them to the configured limit.
esp_err_t drivetrain_kinematics_body_to_wheels(
    const DrivetrainKinematicsConfig *config, 
    const DrivetrainBodyDuty *body, 
    DrivetrainWheelDuty *wheels
);

#ifdef __cplusplus
}
#endif
