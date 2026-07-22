/* Runs the production arm firmware: bridges the Pi link and the drivetrain link. */
#include <Arduino.h>

#include <robot_common/app_log.h>

#include "comms/pi_bridge.h"

static PiBridge bridge = {0};
static bool bridge_ready = false;

// Brings up logging and both UART links (Pi-facing and drivetrain-facing).
void setup() {
    app_log_init();

    esp_err_t err = pi_bridge_init(&bridge);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_UART, "Pi bridge init failed: %s", esp_err_to_name(err));
        return;
    }
    bridge_ready = true;
    APP_LOGI(LOG_TAG_UART, "Pi bridge ready");
}

// Repeatedly drains both links and relays packets between them.
void loop() {
    if (!bridge_ready) {
        delay(100);
        return;
    }

    esp_err_t err = pi_bridge_update(&bridge);
    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_UART, "Pi bridge update failed: %s", esp_err_to_name(err));
    }
    delay(5);   // light throttle -- keeps latency low while yielding to the RTOS
}
