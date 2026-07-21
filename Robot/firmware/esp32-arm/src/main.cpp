/* Runs the arm board's production sensor loop. */
#include <Arduino.h>

#include "config/tof_config.h"
#include "control/time_of_flight/tof_manager.h"

static TofManager tof_manager = {};

// Initializes all three claw sensors and begins continuous ranging.
void setup() {
    ESP_ERROR_CHECK(tof_manager_init(&tof_manager, &ARM_TOF_CONFIG));
    ESP_ERROR_CHECK(tof_manager_start(&tof_manager));
}

// Refreshes each sensor's cached sample without blocking for measurements.
void loop() {
    const esp_err_t error = tof_manager_poll(&tof_manager);
    if (error != ESP_OK && error != ESP_ERR_NOT_FINISHED) {
        ESP_ERROR_CHECK(error);
    }
    delay(1);
}
