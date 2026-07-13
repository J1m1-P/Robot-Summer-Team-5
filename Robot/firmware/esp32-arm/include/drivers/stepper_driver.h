#pragma once

#include <Arduino.h>
#include "config/stepper_config.h"

typedef struct {
    uint8_t stepPin;
    uint8_t dirPin;

    uint32_t stepPulseUs;
    uint32_t stepDelayUs;
} StepperDriver;

// Initialize the stepper driver with the given configuration.
void stepper_begin(StepperDriver *driver, StepperConfig config);

// Functions to control the stepper motor: move a number of steps or a distance in millimeters.
void static stepper_set_direction(StepperDriver *driver, bool direction);
void static stepper_step(StepperDriver *driver);
void stepper_move_steps(StepperDriver *driver, long steps);
void stepper_move_distanceMM(StepperDriver*driver, long distanceMM);

// Setting the step pulse and delay times in microseconds.
void stepper_set_pulse_us(StepperDriver *driver, uint32_t pulseUs);
void stepper_set_delay_us(StepperDriver *driver, uint32_t delayUs);