#pragma once

#include "esp_err.h"
#include "control/drivetrain/drivetrain.h"
#include "control/line_following/line_follower.hpp"
#include "control/line_following/tape_stop_condition.hpp"
#include "control/task/robot_sequence_controller.h"

struct PrecisionMoveContext {
    Drivetrain *drivetrain;
    RobotSequenceController *sequence_controller;
    TapeSensor *sensors[TAPE_SENSOR_MODULE_COUNT];
};

struct PrecisionMoveTarget {
    float dx_body_m;
    float dy_body_m;
    float delta_heading_rad;
    DrivetrainBodyVelocity body_velocity;  // cruise/feed-forward velocity
    bool tape_stop_enabled = false;
    TapeStopSpec tape_stop_spec = {};
    // Optional level-triggered completion condition serviced by the control
    // loop alongside communication, pose, and local hardware inputs.
    const bool *external_stop_requested = nullptr;
    bool world_goal_enabled = false;
    Pose world_goal = {};
};

// Runs a body-relative translation and heading change at 200 Hz. The current
// pose is captured when the action starts; all targets are relative to it.
// The call blocks until settled or timeout, and always stops the drivetrain.
esp_err_t precision_move(const PrecisionMoveContext *context,
                         const PrecisionMoveTarget *target,
                         float timeout_s);

struct TapeAlignContext {
    Drivetrain *drivetrain;
    RobotSequenceController *sequence_controller;
    TapeSensor *sensors[TAPE_SENSOR_MODULE_COUNT];  // PX/front, MX/back, PY/side
};

// Pivots the robot about `travel_dir`'s guide sensor -- approximated at that
// sensor's axle centerpoint from the drivetrain's X-drive geometry -- so it
// stays roughly in place while the body rotates. The correction is driven by
// a second, independently chosen sensor (`feedback_dir`'s sensor), since the
// guide sensor is already on the feature and can't itself reveal body skew:
// the maneuver ends once the tape (or, when `on_gap` is true, the gap between
// two tape edges) is centered on the feedback sensor. Blocks until centered
// or timeout.
esp_err_t align_on_tape(const TapeAlignContext *context, Direction travel_dir,
                        Direction feedback_dir, bool on_gap, float timeout_s);
