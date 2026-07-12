#include <Arduino.h>

#include "esp_err.h"
#include "debug/app_log.h"

#include "communication/i2c/i2c_bus.h"
#include "config/i2c_bus_config.h"

static I2cBus i2c_bus = {0};

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("1. Serial started");

    app_log_init();
    Serial.println("2. App log initialized");

    APP_LOGI(LOG_TAG_I2C, "Starting I2C bus");
    Serial.println("3. First APP_LOGI completed");

    esp_err_t err = i2c_bus_init(
        &i2c_bus,
        &SENSOR_I2C_BUS_CONFIG
    );

    Serial.printf(
        "4. i2c_bus_init returned: %s\n",
        esp_err_to_name(err)
    );

    if (err != ESP_OK) {
        APP_LOGE(
            LOG_TAG_I2C,
            "I2C initialization failed: %s",
            esp_err_to_name(err)
        );
        return;
    }

    Serial.println("5. I2C initialized");

    uint32_t devices_found = 0;

    for (uint8_t address = 0x08; address <= 0x77; address++) {
        err = i2c_bus_probe(&i2c_bus, address);

        if (err == ESP_OK) {
            Serial.printf(
                "Found device at address 0x%02X\n",
                address
            );
            devices_found++;
        }
    }

    Serial.printf(
        "Scan complete. Found %lu device(s)\n",
        (unsigned long)devices_found
    );
}

void loop()
{
    delay(1000);
}