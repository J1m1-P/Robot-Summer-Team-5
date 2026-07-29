/* Coordinates Tower stepper commands and completion status reporting. */
#pragma once

#include <stdint.h>

#include <robot_common/command_packet.h>
#include <robot_common/status_packet.h>
#include <robot_common/uart_link.h>

#include "drivers/stepper_driver.h"

struct TowerActionController {
    UartLink *drivetrain_uart;
    StepperDriver *tower_x_stepper;
    StepperDriver *tower_z_stepper;
    StepperDriver *active_stepper;
    bool action_active;
    bool action_is_timed;
    uint32_t action_complete_ms;
    uint8_t active_command_detail;
    bool locator_extended;
    bool locator_contact_reported;
};

// Connects the controller to initialized UART and stepper hardware.
void tower_action_controller_init(
    TowerActionController *controller,
    UartLink *drivetrain_uart,
    StepperDriver *tower_x_stepper,
    StepperDriver *tower_z_stepper);

// Returns true when the opcode belongs to the Tower action group.
bool tower_action_controller_accepts(CommandOpcode command);

// Returns true while Tower hardware is executing an action.
bool tower_action_controller_is_busy(
    const TowerActionController *controller);

// Starts one decoded Tower command.
void tower_action_controller_start(
    TowerActionController *controller,
    const CommandPacket *command);

// Updates the action and reports completion once over the queued UART link.
bool tower_action_controller_update(
    TowerActionController *controller,
    uint32_t now_ms);
