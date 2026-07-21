#include <Arduino.h>
#include "drivers/stepper_driver.h"

// Stepper motor constants
const float stepAngle = 1.8;
const float stepsPerRev = 360.0 / stepAngle;

// X axis mechanical constants used to convert linear travel into motor steps.
const float beltPitchMM = 2.0;
const uint8_t pulleyTeeth = 20;

// Z axis mechanical constants used to convert linear travel into motor steps.
const float leadscrewPitchMM = 8.0;

// Set the current direction pin state and record the chosen direction.
static void stepper_set_direction(StepperDriver *driver, bool direction) {
    driver->direction = direction;
    digitalWrite(driver->config.dirPin, direction ? HIGH : LOW);
}

// Internal stop helper: clear motion state and lower the step pin.
static void stepper_stop_internal(StepperDriver *driver) {
    driver->isMoving = false;
    driver->stepsRemaining = 0;
    driver->state = 0;
    digitalWrite(driver->config.stepPin, LOW);
}

static bool stepper_is_valid_config(StepperConfig config) {
    if (config.stepPin <= 0 || config.dirPin <= 0 || config.stepPin == config.dirPin) {
        return false;
    }

    if (config.stepPulseUs <= 0 || config.stepDelayUs <= 0 || config.motionlimitMM <= 0) {
        return false;
    }

    if (config.axis != X && config.axis != Z) {
        return false;
    }

    return true;
}

esp_err_t stepper_init(StepperDriver *driver, StepperConfig config) {
    if (!stepper_is_valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    driver->config = config;
    driver->isMoving = false;
    driver->stepsRemaining = 0;
    driver->direction = true;
    driver->lastEventUs = micros();
    driver->pulseStartUs = 0;
    driver->state = 0;

    pinMode(driver->config.stepPin, OUTPUT);
    pinMode(driver->config.dirPin, OUTPUT);

    digitalWrite(driver->config.stepPin, LOW);
    digitalWrite(driver->config.dirPin, LOW);

    return ESP_OK;
}

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

    // Arm the state machine to begin generating pulses in stepper_update().
    driver->stepsRemaining = steps;
    driver->isMoving = true;
    driver->state = 0;
    driver->lastEventUs = micros() - driver->config.stepDelayUs;
    driver->pulseStartUs = 0;
    digitalWrite(driver->config.stepPin, LOW);
}

void stepper_move_distanceMM(StepperDriver *driver, float distanceMM) {
    if (distanceMM == 0) {
        return;
    }

    if (distanceMM > driver->config.motionlimitMM) {
        distanceMM = driver->config.motionlimitMM;
    }
    
    switch (driver->config.axis) {
        case X:
            stepper_x_move_distanceMM(driver, distanceMM);
            break;
        case Z:
            stepper_z_move_distanceMM(driver, distanceMM);
            break;
    }
}

static void stepper_x_move_distanceMM(StepperDriver *driver, float distanceMM) {
    // Convert linear motion to step count using the belt and pulley geometry.
    long steps = (long)round(distanceMM * (stepsPerRev / (beltPitchMM * pulleyTeeth)));
    stepper_move_steps(driver, steps);
}

static void stepper_z_move_distanceMM(StepperDriver *driver, float distanceMM) {
    // Convert linear motion to step count using the leadscrew geometry.
    long steps = (long)round(distanceMM * (stepsPerRev / leadscrewPitchMM));
    stepper_move_steps(driver, steps);
}

void stepper_update(StepperDriver *driver) {
    if (!driver->isMoving || driver->stepsRemaining <= 0) {
        return;
    }

    unsigned long now = micros();

    switch (driver->state) {
        case 0:
            // Wait the configured delay between steps before asserting the next pulse.
            if ((unsigned long)(now - driver->lastEventUs) >= driver->config.stepDelayUs) {
                digitalWrite(driver->config.stepPin, HIGH);
                driver->pulseStartUs = now;
                driver->state = 1;
            }
            break;

        case 1:
            // Keep the step pin high for the configured pulse width.
            if ((unsigned long)(now - driver->pulseStartUs) >= driver->config.stepPulseUs) {
                digitalWrite(driver->config.stepPin, LOW);
                driver->stepsRemaining--;

                if (driver->stepsRemaining <= 0) {
                    stepper_stop_internal(driver);
                } else {
                    driver->lastEventUs = now;
                    driver->state = 0;
                }
            }
            break;

        default:
            // Recover from invalid state by stopping motion.
            stepper_stop_internal(driver);
            break;
    }
}

bool stepper_is_moving(StepperDriver *driver) {
    return driver->isMoving;
}

void stepper_stop(StepperDriver *driver) {
    stepper_stop_internal(driver);
}

void stepper_set_pulse_us(StepperDriver *driver, uint32_t pulseUs) {
    driver->config.stepPulseUs = pulseUs;
}

void stepper_set_delay_us(StepperDriver *driver, uint32_t delayUs) {
    driver->config.stepDelayUs = delayUs;
}
