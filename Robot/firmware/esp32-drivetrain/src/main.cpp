/* Starts and services the drivetrain-side Tower action sequence. */
#include <Arduino.h>

#include <robot_common/uart_link.h>

#include "config/communication/uart_link_config.h"
#include "config/drivetrain/drivetrain_config.h"
#include "control/drivetrain/drivetrain.h"
#include "control/tower_sequence_controller.h"

namespace {

UartLink arm_uart = {};
TowerSequenceController tower_sequence_controller = {};

}  // namespace

// Holds the drivetrain safely, initializes communication, and starts the demo.
void setup() {
    // No drivetrain lifecycle is started for this Tower-only hardware test.
    (void)drivetrain_hold_safe_outputs(&DRIVETRAIN_CONFIG);
    Serial.begin(115200);
    delay(5000);
    Serial.println("# Starting Tower-only action sequence");

    const esp_err_t uart_error =
        uart_link_init(&arm_uart, &TOP_ESP_UART_LINK_CONFIG);
    if (uart_error != ESP_OK) {
        Serial.printf(
            "# DEMO FAULT: arm UART initialization failed (%s)\n",
            esp_err_to_name(uart_error));
        return;
    }

    (void)tower_sequence_controller_init(
        &tower_sequence_controller, &arm_uart);
}

// Services the active Tower action sequence without enabling the drivetrain.
void loop() {
    tower_sequence_controller_service(
        &tower_sequence_controller, millis());
    delay(1);
}
