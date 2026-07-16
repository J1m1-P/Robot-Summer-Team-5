/* Exposes the four drivetrain motor configurations. */
#pragma once

#include "drivers/motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

// Individual motor configurations in physical wheel order.
extern const MotorDriverConfig FL_MOTOR_CONFIG;  // Front Left Motor
extern const MotorDriverConfig FR_MOTOR_CONFIG;  // Front Right Motor
extern const MotorDriverConfig BL_MOTOR_CONFIG;  // Back Left Motor
extern const MotorDriverConfig BR_MOTOR_CONFIG;  // Back Right Motor 

#ifdef __cplusplus
}
#endif
