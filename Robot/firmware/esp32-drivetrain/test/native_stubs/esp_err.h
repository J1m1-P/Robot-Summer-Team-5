#pragma once

// Minimal stand-in for ESP-IDF's esp_err.h, used only by the `native` test
// environment (see platformio.ini) so pure-math modules under test/control
// can be compiled and run on the host without the ESP-IDF/Arduino
// framework. Only the symbols actually used by the code under test are
// defined here -- not a full esp_err.h replacement.

typedef int esp_err_t;

#define ESP_OK 0
#define ESP_ERR_INVALID_ARG 0x102
#define ESP_ERR_INVALID_STATE 0x103
#define ESP_ERR_NOT_SUPPORTED 0x106
#define ESP_ERR_NOT_FOUND 0x105
