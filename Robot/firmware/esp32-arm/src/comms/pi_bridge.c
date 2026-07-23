/*
 * Implements the Pi <-> drivetrain relay. Most COMMAND opcodes (STOP/RESUME/
 * DONE/...) are already unambiguous instructions and pass through unchanged.
 * TURN is the exception: the Pi sends a normalized visual steering error, but
 * the drivetrain needs an angular velocity, so the arm board is the one place
 * that translates between those two units (see turn_error_to_omega() below).
 * STATUS packets need no translation either way and just get relayed as-is.
 */
#include <comms/pi_bridge.h>

#include <robot_common/app_log.h>
#include <robot_common/command_packet.h>
#include <robot_common/status_packet.h>

#include "config/turn_translation_config.h"
#include "config/uart_link_config.h"

// Converts the Pi's normalized visual error (-1..1) into a drivetrain angular
// velocity (rad/s, positive counterclockwise -- see DrivetrainBodyVelocity).
//
// The sign flips deliberately. The Pi's error is POSITIVE when the tubby is to
// the RIGHT of the camera's frame center (see yolo_detect() in
// teletubby_detector.py), which means the robot must turn RIGHT -- clockwise
// -- to center it. Clockwise is NEGATIVE omega in the positive-counterclockwise
// convention x_drive_kinematics.h uses, so this negates the error before
// applying the gain. Get this backwards and the robot turns away from the
// target instead of toward it.
static float turn_error_to_omega(float error) {
    if (error > 1.0f) error = 1.0f;
    if (error < -1.0f) error = -1.0f;

    float omega = -error * TURN_OMEGA_GAIN_RAD_S;
    if (omega > TURN_OMEGA_MAX_RAD_S) omega = TURN_OMEGA_MAX_RAD_S;
    if (omega < -TURN_OMEGA_MAX_RAD_S) omega = -TURN_OMEGA_MAX_RAD_S;
    return omega;
}

// Relays every pending COMMAND packet from the Pi link onto the drivetrain link.
static void relay_commands_to_drivetrain(PiBridge *bridge) {
    PacketFrame frame;
    while (uart_link_take_packet(&bridge->to_pi, &frame) == ESP_OK) {
        CommandPacket command;
        if (command_packet_decode(&frame, &command) != ESP_OK) {
            APP_LOGE(LOG_TAG_UART, "Dropped malformed COMMAND from Pi");
            continue;
        }

        if (command.opcode == CMD_TURN) {
            command.value = turn_error_to_omega(command.value);
        }

        esp_err_t err = command_packet_send(&bridge->to_drivetrain, &command);
        if (err != ESP_OK) {
            APP_LOGE(LOG_TAG_UART, "Failed to relay COMMAND to drivetrain: %s",
                     esp_err_to_name(err));
        }
    }
}

// Relays every pending STATUS packet from the drivetrain link up to the Pi.
static void relay_status_to_pi(PiBridge *bridge) {
    PacketFrame frame;
    while (uart_link_take_packet(&bridge->to_drivetrain, &frame) == ESP_OK) {
        StatusPacket status;
        if (status_packet_decode(&frame, &status) != ESP_OK) {
            APP_LOGE(LOG_TAG_UART, "Dropped malformed STATUS from drivetrain");
            continue;
        }

        esp_err_t err = status_packet_send(&bridge->to_pi, &status);
        if (err != ESP_OK) {
            APP_LOGE(LOG_TAG_UART, "Failed to relay STATUS to Pi: %s",
                     esp_err_to_name(err));
        }
    }
}

esp_err_t pi_bridge_init(PiBridge *bridge) {
    if (bridge == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t err = uart_link_init(&bridge->to_pi, &PI_UART_LINK_CONFIG);
    if (err != ESP_OK) return err;

    err = uart_link_init(&bridge->to_drivetrain, &DRIVETRAIN_UART_LINK_CONFIG);
    if (err != ESP_OK) {
        uart_link_deinit(&bridge->to_pi);
        return err;
    }

    return ESP_OK;
}

esp_err_t pi_bridge_update(PiBridge *bridge) {
    if (bridge == NULL) return ESP_ERR_INVALID_ARG;

    esp_err_t err = uart_link_update(&bridge->to_pi);
    if (err != ESP_OK) return err;

    err = uart_link_update(&bridge->to_drivetrain);
    if (err != ESP_OK) return err;

    relay_commands_to_drivetrain(bridge);
    relay_status_to_pi(bridge);
    return ESP_OK;
}
