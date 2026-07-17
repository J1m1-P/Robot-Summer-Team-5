/* Declares non-blocking step and distance controls for a step/direction motor driver. */
#pragma once

#include <Arduino.h>
#include "config/stepper_config.h"

// Holds the pins, timing values, and runtime state for one initialized stepper driver.
// The state machine in stepper_update() uses these fields to generate pulses without blocking.
typedef struct {
    uint8_t stepPin;          // GPIO pin used for STEP pulses.
    uint8_t dirPin;           // GPIO pin used for direction control.

    uint32_t stepPulseUs;     // Duration of the high step pulse in microseconds.
    uint32_t stepDelayUs;     // Delay between completed pulses in microseconds.

    bool isMoving;            // True when an asynchronous move is active.
    long stepsRemaining;      // Number of steps left to execute.
    bool direction;           // Current movement direction.
    unsigned long lastEventUs;// Timestamp of the last step cycle transition.
    unsigned long pulseStartUs;// Timestamp when the current pulse was asserted.
    uint8_t state;            // Internal state: 0=waiting, 1=pulse active.
} StepperDriver;

// Initialize the stepper driver with the given configuration.
void stepper_begin(StepperDriver *driver, StepperConfig config);

// Starts a signed step movement asynchronously.
// Use stepper_update() from loop() to advance motion.
void stepper_move_steps(StepperDriver *driver, long steps);

// Starts a signed distance movement asynchronously.
// The conversion uses the configured belt and pulley geometry.
void stepper_move_distanceMM(StepperDriver*driver, long distanceMM);

// Advances the stepper state machine; call frequently from loop().
// This function returns immediately and generates pulses over time.
void stepper_update(StepperDriver *driver);

// Returns true while an asynchronous move is active.
bool stepper_is_moving(StepperDriver *driver);

// Stops any active motion and leaves the driver idle.
void stepper_stop(StepperDriver *driver);

// Updates the high time of each step pulse in microseconds.
void stepper_set_pulse_us(StepperDriver *driver, uint32_t pulseUs);

// Updates the delay between step pulses in microseconds.
void stepper_set_delay_us(StepperDriver *driver, uint32_t delayUs);
