/* Defines the servo pin assignments and the two named positions for each servo. */
#pragma once

#include <Arduino.h>
#include <config/pin_map.h>

typedef enum {
    SERVO_POSITION_A = 0,
    SERVO_POSITION_B = 1
} ServoPosition;

struct ServoConfig {
    uint8_t pin;
    uint8_t anglePositionA;
    uint8_t anglePositionB;
    const char *positionNameA;
    const char *positionNameB;
    uint16_t minPulseWidthUs;
    uint16_t maxPulseWidthUs;
    uint16_t frequencyHz;
};

// Both the MG996R and MG90S use a 20 ms frame and standard 1-2 ms pulses.
// Keep these conservative limits unless an individual servo has been calibrated
// with its mechanism disconnected or clear of hard stops.
constexpr uint16_t SERVO_FREQUENCY_HZ = 50;
constexpr uint16_t SERVO_MIN_PULSE_WIDTH_US = 500;
constexpr uint16_t SERVO_MAX_PULSE_WIDTH_US = 2500;

static const ServoConfig habitatLeftServoConfig {PIN_SERVO_HABITAT_LEFT_PWM, 35, 127, "open", "close", SERVO_MIN_PULSE_WIDTH_US, SERVO_MAX_PULSE_WIDTH_US, SERVO_FREQUENCY_HZ};
static const ServoConfig habitatRightServoConfig {PIN_SERVO_HABITAT_RIGHT_PWM, 35, 127, "open", "close", SERVO_MIN_PULSE_WIDTH_US, SERVO_MAX_PULSE_WIDTH_US, SERVO_FREQUENCY_HZ};
static const ServoConfig towerRotateServoConfig {PIN_SERVO_TOWER_ROTATE_PWM, 46, 120, "horizontal", "vertical", SERVO_MIN_PULSE_WIDTH_US, SERVO_MAX_PULSE_WIDTH_US, SERVO_FREQUENCY_HZ};
static const ServoConfig towerLeftServoConfig {PIN_SERVO_TOWER_LEFT_PWM, 50, 89, "open", "close", SERVO_MIN_PULSE_WIDTH_US, SERVO_MAX_PULSE_WIDTH_US, SERVO_FREQUENCY_HZ};
static const ServoConfig towerMiddleServoConfig {PIN_SERVO_TOWER_MIDDLE_PWM, 50, 91, "open", "close", SERVO_MIN_PULSE_WIDTH_US, SERVO_MAX_PULSE_WIDTH_US, SERVO_FREQUENCY_HZ};
static const ServoConfig towerRightServoConfig {PIN_SERVO_TOWER_RIGHT_PWM, 55, 100, "open", "close", SERVO_MIN_PULSE_WIDTH_US, SERVO_MAX_PULSE_WIDTH_US, SERVO_FREQUENCY_HZ};

// Needs calibration: 
static const ServoConfig solarPanelServoConfig {PIN_SERVO_SOLAR_PANEL_PWM, 44, 46, "retract", "extend", SERVO_MIN_PULSE_WIDTH_US, SERVO_MAX_PULSE_WIDTH_US, SERVO_FREQUENCY_HZ};

