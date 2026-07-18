#include <Arduino.h>
#include "drivers/stepper_driver.h"
#include "config/stepper_config.h"

StepperDriver habitatZStepper;
StepperDriver habitatXStepper;
StepperDriver towerZStepper;
StepperDriver towerXStepper;

const long habitatZRangeMM = 152;
const long habitatXRangeMM = 187;
const long towerZRangeMM = 220; // full is 254, but currently interferes with brace
const long towerXRangeMM = 138;

bool movingOutward = true;

static void start_full_range_move(bool outward) {
    stepper_z_move_distanceMM(&habitatZStepper, outward ? -habitatZRangeMM : habitatZRangeMM);
    stepper_x_move_distanceMM(&habitatXStepper, outward ? habitatXRangeMM : -habitatXRangeMM);
    stepper_z_move_distanceMM(&towerZStepper, outward ? -towerZRangeMM : towerZRangeMM);
    stepper_x_move_distanceMM(&towerXStepper, outward ? towerXRangeMM : -towerXRangeMM);
}

void setup() {
    stepper_begin(&habitatZStepper, habitatZConfig);
    stepper_begin(&habitatXStepper, habitatXConfig);
    stepper_begin(&towerZStepper, towerZConfig);
    stepper_begin(&towerXStepper, towerXConfig);

    start_full_range_move(movingOutward);
}

void loop() {
    stepper_update(&habitatZStepper);
    stepper_update(&habitatXStepper);
    stepper_update(&towerZStepper);
    stepper_update(&towerXStepper);

    bool allIdle = !stepper_is_moving(&habitatZStepper) &&
                    !stepper_is_moving(&habitatXStepper) &&
                    !stepper_is_moving(&towerZStepper) &&
                    !stepper_is_moving(&towerXStepper);

    if (allIdle) {
        movingOutward = !movingOutward;
        start_full_range_move(movingOutward);
    }
}