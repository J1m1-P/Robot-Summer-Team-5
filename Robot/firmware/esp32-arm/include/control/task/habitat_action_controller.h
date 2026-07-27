/* Coordinates Habitat stepper commands and completion status reporting. */
#pragma once

#include <stdint.h>

#include <robot_common/command_packet.h>
#include <robot_common/status_packet.h>
#include <robot_common/uart_link.h>

#include "drivers/stepper_driver.h"

struct HabitatActionController {
    UartLink *drivetrain_uart;
    StepperDriver *habitat_x_stepper;
    StepperDriver *habitat_z_stepper;
    StepperDriver *active_stepper;
    bool action_active;
    bool action_is_timed;
    uint32_t action_complete_ms;
    uint8_t active_command_detail;
    bool completion_pending;
    uint32_t repeat_status_until_ms;
    uint32_t last_status_ms;
};

// Connects the controller to initialized UART and stepper hardware.
void habitat_action_controller_init(
    HabitatActionController *controller,
    UartLink *drivetrain_uart,
    StepperDriver *habitat_x_stepper,
    StepperDriver *habitat_z_stepper);

// Returns true when the opcode belongs to the Habitat action group.
bool habitat_action_controller_accepts(CommandOpcode command);

// Returns true while Habitat hardware is executing an action.
bool habitat_action_controller_is_busy(
    const HabitatActionController *controller);

// Starts one decoded Habitat command.
void habitat_action_controller_start(
    HabitatActionController *controller,
    const CommandPacket *command);

// Reports completed actions repeatedly so the drivetrain cannot easily miss
// them. Returns true while completion packets take priority over odometry.
bool habitat_action_controller_update(
    HabitatActionController *controller,
    uint32_t now_ms);
