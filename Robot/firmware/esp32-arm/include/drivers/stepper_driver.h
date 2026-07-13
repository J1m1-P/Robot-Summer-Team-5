#pragma once

#include <Arduino.h>
#include "config/stepper_config.h"

typedef struct {
    uint8_t stepPin;
    uint8_t dirPin;

    uint32_t stepPulseUs;
    uint32_t stepDelayUs;
} StepperDriver;

void stepper_begin(StepperDriver *driver, StepperConfig config);

void stepper_set_direction(StepperDriver *driver, bool direction);
void stepper_step(StepperDriver *driver);
void stepper_move_steps(StepperDriver *driver, long steps);
void stepper_move_distanceMM(StepperDriver*driver, long distanceMM);

void stepper_set_pulse_us(StepperDriver *driver, uint32_t pulseUs);
void stepper_set_delay_us(StepperDriver *driver, uint32_t delayUs);