#include <Arduino.h>
#include "esp_log.h"
#include "esp_err.h"

#include "config/motor_config.h"
#include "config/encoder_config.h"

#include "drivers/motor_driver.h"
#include "drivers/encoder_driver.h"

#include "debug/app_log.h"

static MotorDriver motor = {0};
static EncoderDriver encoder;

void setup()
{
    Serial.begin(115200);
    delay(1000);
    app_log_init();

    encoder_driver_init(&encoder, &FL_ENCODER_CONFIG);
    ESP_LOGI("Encoder", "Encoder initialized with ID: %d", encoder.config.id);

    encoder_driver_start(&encoder);
    ESP_LOGI("Encoder", "Encoder started with ID: %d", encoder.config.id);

    motor_driver_init(&motor, &FL_MOTOR_CONFIG);
    ESP_LOGI("Motor", "Motor initialized: PWM Pin: %d, Direction Pin: %d", motor.config->pwm_pin, motor.config->dir_pin);

    motor_driver_enable(&motor);
    ESP_LOGI("Motor", "Motor enabled");

    // motor_driver_set_duty(&motor, 0.2f);
    // ESP_LOGI("Motor", "Motor duty cycle set to 0.2");

}

void loop()
{   
    encoder_driver_update_velocity(&encoder);
    float mps = encoder_driver_get_velocity_mps(&encoder);
    float rps = encoder_driver_get_velocity_rps(&encoder);
    ESP_LOGI("Encoder current velocity mps", "%f", mps);
    ESP_LOGI("Encoder current velocity rps", "%f", rps);
    motor_driver_set_duty(&motor, 0.1f);

}