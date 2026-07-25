/* Starts and services the drivetrain-side task sequences. */
#include <Arduino.h>

#include <robot_common/uart_link.h>

#include "config/communication/uart_link_config.h"
#include "config/drivetrain/drivetrain_config.h"
#include "control/drivetrain/drivetrain.h"
#include "control/task/tape_following_sequence_controller.h"
#include "control/task/tower_sequence_controller.h"

namespace {

UartLink arm_uart = {};
TowerSequenceController tower_sequence_controller = {};
TapeFollowingSequenceController tape_following_sequence_controller = {};

}  // namespace

void setup() {
    // Must be called first, prevents weird motor behavior at start up
    drivetrain_hold_safe_outputs(&DRIVETRAIN_CONFIG);

    Serial.begin(115200);
    
    // Delay(5000) later need to be change to start button action
    delay(5000); 

    Serial.println("# Starting Task Sequence");

    // UART init
    esp_err_t err = uart_link_init(&arm_uart, &TOP_ESP_UART_LINK_CONFIG); 
    if (err != ESP_OK) {
        Serial.printf("# FAULT: arm UART initialization failed (%s)\n", esp_err_to_name(err));
        return;
    }

    // Tower Building task init
    err = tower_sequence_controller_init(&tower_sequence_controller, &arm_uart);
    if (err != ESP_OK) {
        Serial.printf("# FAULT: Tower initialization failed (%s)\n", esp_err_to_name(err));
        return;
    }

    // Tape Following task init
    err = tape_following_sequence_controller_init(&tape_following_sequence_controller);
    if (err != ESP_OK) {
        Serial.printf("# FAULT: Tape Following initialization failed (%s)\n", esp_err_to_name(err));
        return;
    }
}

void loop() {
    tower_sequence_controller_update(
        &tower_sequence_controller, millis());
    tape_following_sequence_controller_update(
        &tape_following_sequence_controller);
    delay(1);
}
