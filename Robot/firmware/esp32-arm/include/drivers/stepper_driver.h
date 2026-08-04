/* Declares non-blocking step and distance controls for a step/direction motor driver. */
#pragma once

#include <Arduino.h>
#include "config/stepper_config.h"

// Holds the pins, timing values, and hardware-timer state for one stepper.
typedef struct {
    StepperConfig config; // Configuration values for this stepper driver.

    volatile bool isMoving;     // True when an asynchronous move is active.
    volatile long stepsRemaining; // Number of steps left to execute.
    volatile long totalSteps;   // Original move length, used for the speed ramp.
    bool direction;             // Current movement direction.
    volatile bool state;        // Internal state: 0=STEP low, 1=STEP high.
    hw_timer_t *timer;          // Generates STEP edges independently of loop().
    uint8_t timerIndex;         // Hardware timer slot assigned during initialization.
} StepperDriver;

// Initialize the stepper driver with the given configuration.
// Returns ESP_OK on success or an error if configuration/timer allocation fails.
esp_err_t stepper_init(StepperDriver *driver, StepperConfig config);

// Starts a signed step movement asynchronously using the hardware timer.
void stepper_move_steps(StepperDriver *driver, long steps);

// Starts a signed linear movement using the configured axis geometry.
void stepper_move_distanceMM(StepperDriver*driver, float distanceMM);

// Retained for source compatibility; hardware timers now advance motion.
void stepper_update(StepperDriver *driver);

// Returns true while an asynchronous move is active.
bool stepper_is_moving(StepperDriver *driver);

// Stops any active motion and leaves the driver idle.
void stepper_stop(StepperDriver *driver);

// Updates the high time of each step pulse in microseconds.
void stepper_set_pulse_us(StepperDriver *driver, uint32_t pulseUs);

// Updates the cruise delay between step pulses in microseconds.
void stepper_set_delay_us(StepperDriver *driver, uint32_t delayUs);
