/* Defines stepper timing settings and the arm's four stepper configurations. */
#pragma once

#include <Arduino.h>
#include "drivers/stepper_driver.h"

// Defines the pins and pulse timing for one stepper motor.
struct StepperConfig {
    uint8_t stepPin;
    uint8_t dirPin;

    // Step pulse is the time the step pin is held high in microseconds.
    // Step delay is the time between steps in microseconds.
    uint32_t stepPulseUs;
    uint32_t stepDelayUs;
};

// Ready-to-use configurations for the four arm stepper axes.
// Field order is stepPin, dirPin, stepPulseUs, and stepDelayUs.
inline StepperConfig towerXConfig {42, 41, 3, 1000};
inline StepperConfig towerZConfig {15, 16, 3, 1000};
inline StepperConfig habitatXConfig {17, 18, 3, 1000};
inline StepperConfig habitatZConfig {21, 40, 3, 1000};
