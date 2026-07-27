#include <Arduino.h>
#include <driver/timer.h>
#include <hal/gpio_ll.h>
#include "drivers/stepper_driver.h"

namespace {

constexpr uint8_t kHardwareTimerCount = 4;
constexpr uint16_t kTimerDivider = 80;  // 80 MHz APB / 80 = one tick per microsecond.

StepperDriver *timerDrivers[kHardwareTimerCount] = {};

inline timer_group_t IRAM_ATTR timer_group(uint8_t index) {
    return (index & 1U) == 0U ? TIMER_GROUP_0 : TIMER_GROUP_1;
}

inline timer_idx_t IRAM_ATTR timer_number(uint8_t index) {
    return index < 2U ? TIMER_0 : TIMER_1;
}

// Alternates the STEP pin between its configured high pulse and low delay.
bool IRAM_ATTR stepper_timer_interrupt(void *argument) {
    StepperDriver *driver = static_cast<StepperDriver *>(argument);
    if (!driver->isMoving) return false;

    if (driver->state == 0) {
        gpio_ll_set_level(
            &GPIO, static_cast<gpio_num_t>(driver->config.stepPin), HIGH);
        driver->state = 1;
    } else {
        gpio_ll_set_level(
            &GPIO, static_cast<gpio_num_t>(driver->config.stepPin), LOW);
        driver->state = 0;
        if (--driver->stepsRemaining <= 0) {
            driver->isMoving = false;
            return false;
        }
    }

    const uint32_t interval_us =
        driver->state == 1 ? driver->config.stepPulseUs
                           : driver->config.stepDelayUs;
    const timer_group_t group = timer_group(driver->timerIndex);
    const timer_idx_t number = timer_number(driver->timerIndex);
    const uint64_t next_alarm =
        timer_group_get_counter_value_in_isr(group, number) + interval_us;
    timer_group_set_alarm_value_in_isr(group, number, next_alarm);
    timer_group_enable_alarm_in_isr(group, number);
    return false;
}

}  // namespace

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
    const bool physicalDirection = direction != driver->config.directionInverted;
    digitalWrite(driver->config.dirPin, physicalDirection ? HIGH : LOW);
}

// Internal stop helper: clear motion state and lower the step pin.
static void stepper_stop_internal(StepperDriver *driver) {
    timerAlarmDisable(driver->timer);
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
    if (driver == nullptr || !stepper_is_valid_config(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t timer_index = 0;
    while (timer_index < kHardwareTimerCount &&
           timerDrivers[timer_index] != nullptr) {
        ++timer_index;
    }
    if (timer_index == kHardwareTimerCount) return ESP_ERR_NO_MEM;

    driver->config = config;
    driver->isMoving = false;
    driver->stepsRemaining = 0;
    driver->direction = true;
    driver->state = 0;
    driver->timerIndex = timer_index;
    driver->timer = timerBegin(timer_index, kTimerDivider, true);
    if (driver->timer == nullptr) return ESP_FAIL;
    timerDrivers[timer_index] = driver;
    const esp_err_t timer_error = timer_isr_callback_add(
        timer_group(timer_index),
        timer_number(timer_index),
        stepper_timer_interrupt,
        driver,
        ESP_INTR_FLAG_IRAM);
    if (timer_error != ESP_OK) {
        timerDrivers[timer_index] = nullptr;
        timerEnd(driver->timer);
        driver->timer = nullptr;
        return timer_error;
    }
    timerAlarmDisable(driver->timer);

    pinMode(driver->config.stepPin, OUTPUT);
    pinMode(driver->config.dirPin, OUTPUT);

    digitalWrite(driver->config.stepPin, LOW);
    stepper_set_direction(driver, true);

    return ESP_OK;
}

void stepper_move_steps(StepperDriver *driver, long steps) {
    if (steps == 0) {
        return;
    }

    timerAlarmDisable(driver->timer);
    if (steps < 0) {
        stepper_set_direction(driver, false);
        steps = -steps;
    } else {
        stepper_set_direction(driver, true);
    }

    // Start the first pulse now; subsequent edges are hardware timed.
    driver->stepsRemaining = steps;
    driver->isMoving = true;
    driver->state = 1;
    digitalWrite(driver->config.stepPin, HIGH);
    timerRestart(driver->timer);
    timerAlarmWrite(driver->timer, driver->config.stepPulseUs, false);
    timerAlarmEnable(driver->timer);
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

void stepper_update(StepperDriver *driver) {
    (void)driver;
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
