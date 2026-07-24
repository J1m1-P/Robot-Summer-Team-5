/* Coordinates Tower stepper commands and completion status reporting. */
#pragma once

#include <stdint.h>

#include <robot_common/status_packet.h>
#include <robot_common/uart_link.h>

#include "drivers/stepper_driver.h"

struct TowerActionController {
    UartLink *drivetrain_uart;
    StepperDriver *tower_x_stepper;
    StepperDriver *tower_z_stepper;
    StepperDriver *active_stepper;
    bool action_active;
    ActionStatusDetail active_action_detail;
    ActionStatusDetail repeated_action_detail;
    uint32_t repeat_status_until_ms;
    uint32_t last_status_ms;
};

// Connects the controller to hardware owned and initialized by main.cpp.
void tower_action_controller_init(
    TowerActionController *controller,
    UartLink *drivetrain_uart,
    StepperDriver *tower_x_stepper,
    StepperDriver *tower_z_stepper);

// Receives one queued command and starts the requested Tower movement.
void tower_action_controller_service_commands(
    TowerActionController *controller);

// Reports completed actions repeatedly so the drivetrain cannot easily miss
// them. Returns true while completion packets take priority over odometry.
bool tower_action_controller_service_status(
    TowerActionController *controller,
    uint32_t now_ms);
