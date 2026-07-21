/* Defines stepper timing settings and the arm's four stepper configurations. */
#pragma once

#include <Arduino.h>

// Defines the pins and pulse timing for one stepper motor.
struct ServoConfig {
    uint8_t servoPin;

    // minAngle and maxAngle define the range of motion for the servo in degrees. 
    uint16_t minAngle; 
    uint16_t maxAngle;
};

// Ready-to-use configurations for the six servos.
// Field order is stepPin, dirPin, stepPulseUs, stepDelayUs, and motionlimitMM.
inline ServoConfig habitatLeftConfig {16, 15, 3};
inline ServoConfig habitatRightConfig {42, 41, 3};
inline ServoConfig towerRotateConfig {18, 17, 90};
inline ServoConfig towerLeftConfig {21, 40, 90};
inline ServoConfig towerMiddleConfig {21, 40, 90};
inline ServoConfig towerRightConfig {21, 40, 90};