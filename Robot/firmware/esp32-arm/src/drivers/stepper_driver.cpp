/* Implements blocking pulse generation for step/direction motor drivers. */
#include "drivers/stepper_driver.h"

// Mechanical constants used to convert linear travel into motor steps.
const float stepAngle = 1.8;
const float beltPitchMM = 2.0; 
const uint8_t pulleyTeeth = 20;
const float stepsPerRev = 360/stepAngle; 

// Copies configuration into the runtime driver and initializes its output pins.
void stepper_begin(StepperDriver *driver, StepperConfig config) {
    driver->stepPin = config.stepPin;
    driver->dirPin = config.dirPin;
    driver->stepPulseUs = config.stepPulseUs;
    driver->stepDelayUs = config.stepDelayUs;

    pinMode(driver->stepPin, OUTPUT);
    pinMode(driver->dirPin, OUTPUT);

    digitalWrite(driver->stepPin, LOW);
    digitalWrite(driver->dirPin, LOW);
}

// Set the direction of the stepper by setting true or false.
// The actual direction needs to be physically verified. 
void static stepper_set_direction(StepperDriver *driver, bool direction) {
    digitalWrite(driver->dirPin, direction ? HIGH : LOW);
}

// Drive the stepper by one step in the set direction. 
void static stepper_step(StepperDriver *driver) {
    digitalWrite(driver->stepPin, HIGH);
    delayMicroseconds(driver->stepPulseUs);

    digitalWrite(driver->stepPin, LOW);
    delayMicroseconds(driver->stepDelayUs);
}

// Drive the stepper by a number of steps. 
// Negative steps will move in the opposite direction.
void stepper_move_steps(StepperDriver *driver, long steps) {
    if (steps == 0) {
        return;
    }

    if (steps < 0) {
        stepper_set_direction(driver, false);
        steps = -steps;
    } else {
        stepper_set_direction(driver, true);
    }

    for (long i = 0; i < steps; i++) {
        stepper_step(driver);
    }
}

// Drive the stepper by a distance in millimeters.
// Negative distance will move in the opposite direction.
void stepper_move_distanceMM(StepperDriver *driver, long distanceMM) {
    if (distanceMM == 0) {
        return;
    }

    if (distanceMM < 0) {
        stepper_set_direction(driver, false);
        distanceMM = -distanceMM;
    } else {
        stepper_set_direction(driver, true);
    }

    // Calculate number of steps for a distance
    long steps = distanceMM * (stepsPerRev / (beltPitchMM * pulleyTeeth));
    stepper_move_steps(driver, steps);
}

// Updates the duration of the high portion of each step pulse.
void stepper_set_pulse_us(StepperDriver *driver, uint32_t pulseUs) {
    driver->stepPulseUs = pulseUs;
}

// Updates the delay inserted after each step pulse.
void stepper_set_delay_us(StepperDriver *driver, uint32_t delayUs) {
    driver->stepDelayUs = delayUs;
}
