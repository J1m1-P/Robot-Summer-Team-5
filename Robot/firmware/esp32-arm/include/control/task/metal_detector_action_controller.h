/* Coordinates metal detector baseline-set and read commands over UART. */
#pragma once

#include <stdint.h>

#include <robot_common/command_packet.h>
#include <robot_common/uart_link.h>

#include "drivers/metal_detector_driver.h"

struct MetalDetectorActionController {
    UartLink *drivetrain_uart;
    MetalDetectorDriver *detector;
    bool report_pending;
    uint8_t pending_detail;
};

// Connects the controller to the initialized UART and detector driver.
void metal_detector_action_controller_init(
    MetalDetectorActionController *controller,
    UartLink *drivetrain_uart,
    MetalDetectorDriver *detector);

// Returns true when the opcode belongs to the metal detector action group.
bool metal_detector_action_controller_accepts(CommandOpcode command);

// Always false: both actions complete synchronously within start().
bool metal_detector_action_controller_is_busy(
    const MetalDetectorActionController *controller);

// Executes one decoded metal detector command immediately and queues its
// completion status for the next update().
void metal_detector_action_controller_start(
    MetalDetectorActionController *controller,
    const CommandPacket *command);

// Keeps the detector's sample window paced and reports one queued
// completion status per call over the shared UART link.
bool metal_detector_action_controller_update(
    MetalDetectorActionController *controller,
    uint32_t now_ms);
