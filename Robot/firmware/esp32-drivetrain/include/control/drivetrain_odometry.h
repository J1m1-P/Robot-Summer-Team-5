#pragma once

#include "esp_err.h"

// Cumulative world-frame pose tracking for this board.
//
// esp32-arm's PMW3610 fusion sends one body-frame motion delta per cycle
// (forward_mm, lateral_mm, heading_delta_rad, valid) over UART as a
// PACKET_TYPE_ODOMETRY packet (see esp32-arm/optical_readme.md and
// esp32-arm/include/comm/packets/delta_pose_packet.h). esp32-arm keeps its
// own cumulative pose too, but only as a bench-debug convenience that
// resets to the origin on any sensor fault -- per that header's own
// comment, THIS board is meant to own the real fault policy and the
// canonical cumulative pose. This module is that owner.
//
// Not wired to UART yet: esp32-drivetrain has no comm/uart_link or
// delta_pose_packet module ported in yet, so for now this only defines the
// integration step (delta-in, pose-out). Decoding incoming UART frames
// into DrivetrainOdometryDelta is a separate task.

struct DrivetrainOdometryDelta {
    float forward_mm = 0.0f;        // robot-frame +x (forward) displacement this cycle
    float lateral_mm = 0.0f;        // robot-frame +y (strafe right) displacement this cycle
    float heading_delta_rad = 0.0f; // robot-frame heading change this cycle, +CCW
};

struct DrivetrainPose {
    float x_mm = 0.0f;
    float y_mm = 0.0f;
    float heading_rad = 0.0f;
};

// TODO: decide the real fault policy (this is the whole point of owning
// it here instead of on esp32-arm). Candidates: latch until explicitly
// cleared vs. esp32-arm's auto-reset-and-resume; whether an invalid cycle
// should hold pose steady instead of zeroing it; how many consecutive
// invalid cycles before declaring the estimate untrustworthy.
enum class DrivetrainOdometryFault {
    None,
    InvalidCycle,
};

struct DrivetrainOdometry {
    DrivetrainPose pose;

    bool fault_latched = false;
    DrivetrainOdometryFault last_fault = DrivetrainOdometryFault::None;

    // Default-constructed = origin, no fault. No separate init()/reset()
    // pair needed: `DrivetrainOdometry odo;` starts clean, and
    // `odo = DrivetrainOdometry{};` resets it -- see
    // drivetrain_odometry_reset() below, which is just that assignment.
};

// Integrate one cycle's body-frame delta into the cumulative world-frame
// pose using the world<-body rotation by the current heading:
//
//   R(theta) = [[cos(theta), sin(theta), 0],
//               [-sin(theta), cos(theta), 0],
//               [0, 0, 1]]
//   [vx, vy, omega]^T = R(theta) * [x_a_dot, y_a_dot, theta_dot]^T
//
// `valid` mirrors the UART packet's valid flag (false when either
// upstream optical sensor read was bad this cycle) -- see the fault
// policy TODO above for what should happen to odo.pose when false.
esp_err_t drivetrain_odometry_update(DrivetrainOdometry &odo, const DrivetrainOdometryDelta &delta, bool valid);

void drivetrain_odometry_reset(DrivetrainOdometry &odo);
