/* Coordinates Raspberry Pi commands and report forwarding over UART. */
#pragma once

#include <stdint.h>

#include <robot_common/command_packet.h>
#include <robot_common/pi_action_packet.h>
#include <robot_common/uart_link.h>

struct PiActionController {
    UartLink *pi_uart;
    UartLink *drivetrain_uart;
    bool action_active;
    uint8_t request_id;
    uint32_t response_deadline_ms;
};

// Connects the dedicated Pi UART and the shared drivetrain UART.
void pi_action_controller_init(
    PiActionController *controller,
    UartLink *pi_uart,
    UartLink *drivetrain_uart);

// Returns true when the opcode belongs to the Raspberry Pi action group.
bool pi_action_controller_accepts(CommandOpcode command);

// Returns true while the controller is waiting for the Pi's report.
bool pi_action_controller_is_busy(const PiActionController *controller);

// Sends one decoded Pi command and begins waiting for its matching report.
void pi_action_controller_start(
    PiActionController *controller,
    const CommandPacket *command,
    uint32_t now_ms);

// Services the Pi UART and relays one validated report to the drivetrain.
bool pi_action_controller_update(
    PiActionController *controller,
    uint32_t now_ms);
