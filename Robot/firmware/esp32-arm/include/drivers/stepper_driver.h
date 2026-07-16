/* Declares blocking step and distance controls for a step/direction motor driver. */
#pragma once

#include <Arduino.h>
#include "config/stepper_config.h"

// Holds the pins and active timing values for one initialized stepper driver.
typedef struct {
    uint8_t stepPin;
    uint8_t dirPin;

    uint32_t stepPulseUs;
    uint32_t stepDelayUs;
} StepperDriver;

// Initialize the stepper driver with the given configuration.
void stepper_begin(StepperDriver *driver, StepperConfig config);

// Sets the direction pin used by subsequent step pulses.
void static stepper_set_direction(StepperDriver *driver, bool direction);

// Generates one blocking pulse on the step pin.
void static stepper_step(StepperDriver *driver);

// Moves a signed number of steps using blocking pulse generation.
void stepper_move_steps(StepperDriver *driver, long steps);

// Converts a signed distance in millimeters to a blocking step movement.
void stepper_move_distanceMM(StepperDriver*driver, long distanceMM);

// Updates the high time of each step pulse in microseconds.
void stepper_set_pulse_us(StepperDriver *driver, uint32_t pulseUs);

// Updates the delay between step pulses in microseconds.
void stepper_set_delay_us(StepperDriver *driver, uint32_t delayUs);
