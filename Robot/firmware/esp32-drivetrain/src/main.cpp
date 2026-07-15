#include <Arduino.h>

#include "esp_err.h"
#include <robot_common/app_log.h>

#include "drivers/motor_driver.h"
#include "config/motor_config.h"

static MotorDriver motor1 = {0};

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
        APP_LOGE(LOG_TAG_MOTOR, "Failed motor initlization");
    }
    APP_LOGI(LOG_TAG_MOTOR, "Motor intialized");

    err = motor_driver_enable(&motor1);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_MOTOR, "Failed motor enable");
    }
    APP_LOGI(LOG_TAG_MOTOR, "Motor enabled");

}

void loop()
{
    motor_driver_set_duty(&motor1, 0.5);
    delay(100);

}
