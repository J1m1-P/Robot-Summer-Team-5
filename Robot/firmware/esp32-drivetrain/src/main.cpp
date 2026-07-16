/* Runs the current drivetrain motor bench-test firmware. */
#include <Arduino.h>

#include "esp_err.h"
#include <robot_common/app_log.h>

#include "drivers/motor_driver.h"
#include "config/motor_config.h"

// Runtime state for the single-motor bench test.
static MotorDriver motor1 = {0};
static bool motor_ready = false;

// Initializes logging and enables the front-left motor for bench testing.
void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("1. Serial started");

    app_log_init();
    Serial.println("2. App log initialized");

    esp_err_t err; 

    err = motor_driver_init(&motor1, &FL_MOTOR_CONFIG);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_MOTOR, "Failed motor initialization: %s", esp_err_to_name(err));
        return;
    }
    APP_LOGI(LOG_TAG_MOTOR, "Motor initialized");

    err = motor_driver_enable(&motor1);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_MOTOR, "Failed motor enable: %s", esp_err_to_name(err));
        return;
    }
    APP_LOGI(LOG_TAG_MOTOR, "Motor enabled");
    motor_ready = true;
}

// Repeatedly commands a fixed duty until a motor error occurs.
void loop()
{
    if (!motor_ready) {
        delay(100);
        return;
    }

    esp_err_t err = motor_driver_set_duty(&motor1, 0.5f);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_MOTOR, "Failed to set motor duty: %s", esp_err_to_name(err));
        motor_ready = false;
    }
    delay(100);
}
