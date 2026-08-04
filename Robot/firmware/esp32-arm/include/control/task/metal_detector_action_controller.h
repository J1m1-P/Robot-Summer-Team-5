/* Coordinates the rock arm, claw, and metal detector workflow. */
#pragma once

#include <stdint.h>

#include <robot_common/command_packet.h>
#include <robot_common/status_packet.h>
#include <robot_common/uart_link.h>

#include "drivers/metal_detector_driver.h"

enum MetalDetectorActionStage : uint8_t {
    METAL_ACTION_IDLE = 0,
    METAL_ACTION_WAITING_FOR_SEMI_LOWER,
    METAL_ACTION_WAITING_FOR_CLAW_OPEN,
    METAL_ACTION_SAMPLING_BASELINE,
    METAL_ACTION_WAITING_FOR_LOWER,
    METAL_ACTION_SAMPLING_ROCK,
    METAL_ACTION_WAITING_FOR_CLEARANCE,
    METAL_ACTION_WAITING_FOR_CLAW_CLOSE,
    METAL_ACTION_WAITING_FOR_FULL_LIFT,
};

struct MetalDetectorActionController {
    UartLink *drivetrain_uart;
    MetalDetectorDriver *detector;
    MetalDetectorActionStage stage;
    uint32_t stage_complete_ms;
    StatusCode result_code;
    uint8_t result_detail;
    bool report_pending;
};

// Connects the controller and initializes the rock lift and claw servos.
void metal_detector_action_controller_init(
    MetalDetectorActionController *controller,
    UartLink *drivetrain_uart,
    MetalDetectorDriver *detector);

// Returns true for the baseline and rock-sampling commands.
bool metal_detector_action_controller_accepts(CommandOpcode command);

// Returns true while the workflow is moving, sampling, or reporting.
bool metal_detector_action_controller_is_busy(
    const MetalDetectorActionController *controller);

// Starts the preparation or rock-sampling half of the workflow.
void metal_detector_action_controller_start(
    MetalDetectorActionController *controller,
    const CommandPacket *command);

// Advances servo and detector stages and reports completion once.
bool metal_detector_action_controller_update(
    MetalDetectorActionController *controller,
    uint32_t now_ms);
