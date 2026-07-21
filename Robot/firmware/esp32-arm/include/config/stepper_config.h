/* Defines stepper timing settings and the arm's four stepper configurations. */
#pragma once

#include <Arduino.h>
#include <config/pin_map.h>

// Axis identifiers for stepper_move_distanceMM().
typedef enum {X, Z} Axis;

// Defines the pins and pulse timing for one stepper motor.
struct StepperConfig {
    uint8_t stepPin;
    uint8_t dirPin;

    // Step pulse is the time the step pin is held high in microseconds.
    // Step delay is the time between steps in microseconds.
    uint32_t stepPulseUs;
    uint32_t stepDelayUs;

    // Maximum distance in millimeters that the stepper is allowed to move in one direction.        
    uint32_t motionlimitMM; 

    Axis axis; // Axis identifier for this stepper (X or Z).
};

// Ready-to-use configurations for the four arm stepper axes.
// Field order is stepPin, dirPin, stepPulseUs, stepDelayUs, motionlimitMM, and axis.
inline StepperConfig towerXConfig {PIN_STEP1, PIN_STEP1_DIR, 3, 3000, 138, X};
inline StepperConfig towerZConfig {PIN_STEP2, PIN_STEP2_DIR, 3, 1500, 220, Z};
inline StepperConfig habitatXConfig {PIN_STEP3, PIN_STEP3_DIR, 3, 3000, 187, X};
inline StepperConfig habitatZConfig {PIN_STEP4, PIN_STEP4_DIR, 3, 1500, 152, Z};