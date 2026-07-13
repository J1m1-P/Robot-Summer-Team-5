// lib/stepper-driver/src/stepper-driver.cpp

#include "drivers/stepper_driver.h"

const float stepAngle = 1.8;
const float beltPitchMM = 2.0; 
const uint8_t pulleyTeeth = 20;
const float stepsPerRev = 360/stepAngle; 

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
void stepper_set_direction(StepperDriver *driver, bool direction) {
    digitalWrite(driver->dirPin, direction ? HIGH : LOW);
}

// Drive the stepper by one step. 
void stepper_step(StepperDriver *driver) {
    digitalWrite(driver->stepPin, HIGH);
    delayMicroseconds(driver->stepPulseUs);

    digitalWrite(driver->stepPin, LOW);
    delayMicroseconds(driver->stepDelayUs);
}

// Drive the stepper by a number of steps. 
void stepper_move_steps(StepperDriver *driver, long steps) {
    if (steps == 0) {
        return;
    }

    for (long i = 0; i < steps; i++) {
        stepper_step(driver);
    }
}

// Drive the stepper by a number of steps. 
void stepper_move_distanceMM(StepperDriver *driver, long distanceMM) {
    if (distanceMM == 0) {
        return;
    }
    
    // Calculate number of steps for a distance
    long steps = distanceMM * (stepsPerRev / (beltPitchMM * pulleyTeeth));
    stepper_move_steps(driver, steps);
}

void stepper_set_pulse_us(StepperDriver *driver, uint32_t pulseUs) {
    driver->stepPulseUs = pulseUs;
}

void stepper_set_delay_us(StepperDriver *driver, uint32_t delayUs) {
    driver->stepDelayUs = delayUs;
}