#pragma once

#include <Arduino.h>
#include <ESP32Servo.h>
#include "config/servo_config.h"

struct ServoDriver {
    ServoConfig config;
    Servo servo;
    bool attached = false;
    ServoPosition currentPosition = SERVO_POSITION_A;
};

esp_err_t servo_init(ServoDriver *driver, const ServoConfig &config);
void servo_set_position(ServoDriver *driver, ServoPosition position);
bool servo_set_position_by_name(ServoDriver *driver, const char *name);
void servo_set_angle(ServoDriver *driver, uint8_t angle);

ServoPosition servo_get_current_position(const ServoDriver *driver);
const char *servo_get_current_position_name(const ServoDriver *driver);
