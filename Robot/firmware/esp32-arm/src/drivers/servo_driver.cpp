#include <Arduino.h>
#include <string.h>
#include "drivers/servo_driver.h"

static bool servo_config_is_valid(const ServoConfig &config) {
    if (config.minPulseWidthUs < MIN_PULSE_WIDTH
        || config.maxPulseWidthUs > MAX_PULSE_WIDTH
        || config.minPulseWidthUs >= config.maxPulseWidthUs
        || config.frequencyHz == 0
        || config.maxPulseWidthUs >= (1000000UL / config.frequencyHz)) {
        return false;
    }

    if (config.anglePositionA > 180 || config.anglePositionB > 180
        || config.positionNameA == nullptr || config.positionNameB == nullptr) {
        return false;
    }
    return true;
}

static bool servo_driver_is_valid(const ServoDriver *driver) {
    return driver != nullptr && driver->attached;
}

esp_err_t servo_init(ServoDriver *driver, const ServoConfig &config) {
    if (driver == nullptr || !servo_config_is_valid(config)) {
        return ESP_ERR_INVALID_ARG;
    }

    driver->config = config;
    driver->attached = false;

    driver->servo.setPeriodHertz(config.frequencyHz);
    driver->servo.attach(
        config.pin,
        config.minPulseWidthUs,
        config.maxPulseWidthUs);
    driver->attached = driver->servo.attached();

    if (driver->attached) {
        driver->servo.write(config.anglePositionA);
        driver->currentPosition = SERVO_POSITION_A;
        return ESP_OK;
    } else {
        return ESP_FAIL;
    }
}

void servo_set_position(ServoDriver *driver, ServoPosition position) {
    if (!servo_driver_is_valid(driver)
        || (position != SERVO_POSITION_A && position != SERVO_POSITION_B)) {
        return;
    }

    uint8_t targetAngle = (position == SERVO_POSITION_B)
        ? driver->config.anglePositionB
        : driver->config.anglePositionA;

    driver->servo.write(constrain(targetAngle, 0, 180));
    driver->currentPosition = position;
}

bool servo_set_position_by_name(ServoDriver *driver, const char *name) {
    if (!servo_driver_is_valid(driver) || name == nullptr) {
        return false;
    }

    if (strcmp(name, driver->config.positionNameA) == 0) {
        servo_set_position(driver, SERVO_POSITION_A);
        return true;
    }

    if (strcmp(name, driver->config.positionNameB) == 0) {
        servo_set_position(driver, SERVO_POSITION_B);
        return true;
    }

    return false;
}

void servo_set_angle(ServoDriver *driver, uint8_t angle) {
    if (!servo_driver_is_valid(driver)) {
        return;
    }

    driver->servo.write(constrain(angle, 0, 180));
}

const char *servo_get_current_position_name(const ServoDriver *driver) {
    if (!servo_driver_is_valid(driver)) {
        return nullptr;
    }

    return (driver->currentPosition == SERVO_POSITION_B)
        ? driver->config.positionNameB
        : driver->config.positionNameA;
}

ServoPosition servo_get_current_position(const ServoDriver *driver) {
    if (!servo_driver_is_valid(driver)) {
        return SERVO_POSITION_A;
    }

    return driver->currentPosition;
}
