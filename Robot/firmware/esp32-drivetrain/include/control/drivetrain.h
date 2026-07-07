#pragma once 

#include "drivers/motor_driver.h"

typedef struct {
    MotorDriver *FL_motor;
    MotorDriver *FR_motor;
    MotorDriver *BL_motor;
    MotorDriver *BR_motor;
}