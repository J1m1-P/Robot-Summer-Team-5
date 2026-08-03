/* Routes shared commands to the Tower, Habitat, or Pi action controller. */
#pragma once

#include <stdint.h>

#include <robot_common/uart_link.h>

#include "control/task/habitat_action_controller.h"
#include "control/task/tower_action_controller.h"
#include "control/task/pi_action_controller.h"

struct ArmActionDispatcher {
    UartLink *drivetrain_uart;
    TowerActionController *tower_controller;
    HabitatActionController *habitat_controller;
    PiActionController *pi_controller;
    bool readiness_pending;
    bool uart_fault_logged;
    uint32_t last_ready_status_ms;
    uint32_t last_solar_panel_contact_status_ms;
};

// Connects the shared UART to all arm-side action controllers.
void arm_action_dispatcher_init(
    ArmActionDispatcher *dispatcher,
    UartLink *drivetrain_uart,
    TowerActionController *tower_controller,
    HabitatActionController *habitat_controller,
    PiActionController *pi_controller);

// Services one command and both controllers. Returns true while arm status
// packets take priority over odometry on the shared UART.
bool arm_action_dispatcher_update(
    ArmActionDispatcher *dispatcher,
    uint32_t now_ms);
