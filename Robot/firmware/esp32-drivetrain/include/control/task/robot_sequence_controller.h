/* Coordinates one ordered sequence of drivetrain and arm actions. */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include <robot_common/packet_protocol.h>
#include <robot_common/uart_link.h>

#include "control/task/movement_action_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    UartLink *arm_uart;
    MovementActionController movement_action_controller;
    size_t current_step;
    uint32_t step_deadline_ms;
    bool running;
    bool waiting_for_arm_ready;
    float scan_rotation_degrees;
    uint8_t last_pi_request_id;
} RobotSequenceController;

// Connects the action paths and waits for the arm before starting the sequence.
esp_err_t robot_sequence_controller_init(
    RobotSequenceController *controller,
    UartLink *arm_uart);

// Updates only the current step and advances when it completes.
void robot_sequence_controller_update(
    RobotSequenceController *controller,
    uint32_t now_ms);

// Processes one arm_uart frame the caller has already dequeued. arm_uart is
// shared with odometry traffic, so this controller no longer reads it
// itself -- see comm/pose_service.h for the single reader/dispatcher.
void robot_sequence_controller_handle_frame(
    RobotSequenceController *controller,
    const PacketFrame *frame,
    uint32_t now_ms);

#ifdef __cplusplus
}
#endif
