/* Coordinates metal detector baseline-set and read commands over UART. */
#pragma once

#include <stdint.h>

#include <robot_common/command_packet.h>
#include <robot_common/status_packet.h>
#include <robot_common/uart_link.h>

#include "drivers/metal_detector_driver.h"

struct MetalDetectorActionController {
    UartLink *drivetrain_uart;
    MetalDetectorDriver *detector;
    CommandOpcode active_command;
    bool action_active;
    bool report_pending;
    StatusCode pending_code;
    uint8_t pending_detail;
};

// Connects the controller to the initialized UART and detector driver.
void metal_detector_action_controller_init(
    MetalDetectorActionController *controller,
    UartLink *drivetrain_uart,
    MetalDetectorDriver *detector);

// Returns true when the opcode belongs to the metal detector action group.
bool metal_detector_action_controller_accepts(CommandOpcode command);

// True while a fresh sample is being collected or its result awaits sending.
bool metal_detector_action_controller_is_busy(
    const MetalDetectorActionController *controller);

// Starts a fresh detector sample for one decoded command.
void metal_detector_action_controller_start(
    MetalDetectorActionController *controller,
    const CommandPacket *command);

// Polls the active sample and reliably reports its result over the shared UART.
bool metal_detector_action_controller_update(
    MetalDetectorActionController *controller,
    uint32_t now_ms);
