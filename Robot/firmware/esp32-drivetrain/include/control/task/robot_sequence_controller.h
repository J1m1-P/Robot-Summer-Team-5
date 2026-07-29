/* Coordinates one ordered sequence of drivetrain and arm actions. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include <robot_common/uart_link.h>

#include "comm/odometry_link.h"
#include "control/drivetrain/drivetrain.h"
#include "control/odometry/pose_tracker.h"
#include "control/task/movement_action_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    PoseTracker *pose_tracker;
    Drivetrain *drivetrain;
    UartLink *arm_uart;
    Pmw3610OdometryLink *odometry_link;
    MovementActionController movement_action_controller;
    size_t current_step;
    uint32_t step_deadline_ms;
    bool running;
    bool waiting_for_arm_ready;
    bool locator_contact_pending;
    bool updating_movement;
    uint8_t last_pi_request_id;
    uint8_t align_attempts;   // scan+rotate rounds used on the current PI_ALIGN step
    float last_rotation_degrees;  // most recent rotation commanded, for NOT_FOUND recovery
    float chase_net_rotation_degrees;  // sum of every rotation actually commanded
                                        // since the current chase started -- what
                                        // REPOSITION/ALL_FOUND undo, instead of
                                        // trusting the Pi's reported error
} RobotSequenceController;

// Initializes the sequence; call robot_sequence_controller_start() after
// movement contexts are connected.
esp_err_t robot_sequence_controller_init(
    RobotSequenceController *controller,
    PoseTracker *pose_tracker,
    Drivetrain *drivetrain,
    UartLink *arm_uart,
    Pmw3610OdometryLink *odometry_link);

// Starts the current sequence step without waiting for an arm-ready frame.
esp_err_t robot_sequence_controller_start(
    RobotSequenceController *controller,
    uint32_t now_ms);

// One control-cycle entry point. It drains arm UART, updates pose, then
// advances the current sequence step. Blocking movements call this same
// function; nested calls service UART/pose without recursively running the
// movement step.
esp_err_t robot_sequence_controller_update(
    RobotSequenceController *controller,
    uint32_t now_ms);

#ifdef __cplusplus
}
#endif
