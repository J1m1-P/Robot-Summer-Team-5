#include <Arduino.h>
#include "esp_log.h"
#include "esp_err.h"


#include "config/motor_config.h"
#include "drivers/motor_driver.h"
#include "debug/app_log.h"

static MotorDriver motor = {0};

void setup()
{
    Serial.begin(115200);
    delay(1000);
    app_log_init();

    APP_LOGI(LOG_TAG_MAIN, "Starting ESP32 Drivetrain Firmware");

    motor_driver_init(&motor, &FL_MOTOR_CONFIG);

    APP_LOGI(LOG_TAG_MAIN, "Motor initialized: %s", motor_driver_is_initialized(&motor) ? "true" : "false");

    motor_driver_enable(&motor);

    APP_LOGI(LOG_TAG_MAIN, "Motor enabled: %s", motor_driver_is_enabled(&motor) ? "true" : "false");

}

void loop()
{
    motor_driver_set_duty(&motor, 0.5);
    APP_LOGI(LOG_TAG_MAIN, "Motor duty set to %0.1f", motor_driver_get_current_duty(&motor));
    delay(2000);

    motor_driver_set_duty(&motor, -0.5);
    APP_LOGI(LOG_TAG_MAIN, "Motor duty set to %0.1f", motor_driver_get_current_duty(&motor));
    delay(2000);
}