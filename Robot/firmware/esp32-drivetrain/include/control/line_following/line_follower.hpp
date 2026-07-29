#ifndef LINE_FOLLOWER_HPP
#define LINE_FOLLOWER_HPP

#include "control/drivetrain/drivetrain.h"
#include "control/line_following/tape_stop_condition.hpp"
#include "control/task/robot_sequence_controller.h"
#include "drivers/tape_sensor/tape_sensor_driver.h"

enum class Direction { PX, MX, PY };

// LATERAL_ONE stops on first detection; LATERAL_TWO stops on a full outer-tape
// / centre-gap pattern. PX uses side, PY uses back, and MX has no -y sensor.
enum class StopCondition { LATERAL_ONE, LATERAL_TWO, DISTANCE, TIME_ONLY };

// Caller owns/initializes these dependencies once and reuses the context.
// The controller is borrowed and remains live during blocking maneuvers.
struct LineFollowerContext {
    Drivetrain *drivetrain;
    TapeSensor *sensors[3];  // front, back, side
    RobotSequenceController *sequence_controller;
};

// Runs tape-following until `stop_type` is satisfied or `timeout_s` elapses.
// Blocks the calling task for the duration of the maneuver. Returns true if
// the stop condition was reached, false on timeout, lost tape, or error.
// The approach ramp (see kApproachRampDistanceM) brings speed down before a
// known DISTANCE stop point, but there is no post-stop overshoot
// correction -- callers should not assume the true stop point is hit exactly.
bool follow_tape(LineFollowerContext *ctx, Direction dir, float speed_mps,
                  StopCondition stop_type, float stop_value, float timeout_s);

#endif
