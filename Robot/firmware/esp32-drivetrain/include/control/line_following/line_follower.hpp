#ifndef LINE_FOLLOWER_HPP
#define LINE_FOLLOWER_HPP

#include "control/drivetrain/drivetrain.h"
#include "control/line_following/tape_stop_condition.hpp"
#include "control/task/robot_sequence_controller.h"
#include "drivers/tape_sensor/tape_sensor_driver.h"

enum class Direction { PX, MX, PY };

// SINGLE_SENSOR preserves the original behavior. FRONT_BACK_ALIGNED uses the
// front and back modules together and is valid for PX/MX travel.
enum class TapeFollowMode { SINGLE_SENSOR, FRONT_BACK_ALIGNED };

// RISE_ONE stops on one detected tape edge; RISE_TWO waits for two tape edges
// with a gap between them. For marker stops, PX uses the side/PY module,
// PY uses the back/MX module, and MX has no lateral marker module.
enum class StopCondition { RISE_ONE, RISE_TWO, DISTANCE, TIME_ONLY };

// Caller owns/initializes these dependencies once and reuses the context.
// The controller is borrowed and remains live during blocking maneuvers.
struct LineFollowerContext {
    Drivetrain *drivetrain;
    TapeSensor *sensors[3];  // PX/front, MX/back, PY/side
    RobotSequenceController *sequence_controller;
};

// Runs tape-following until `stop_type` is satisfied or `timeout_s` elapses.
// Blocks the calling task for the duration of the maneuver. Returns true if
// the stop condition was reached, false on timeout, lost tape, or error.
// The approach ramp (see kApproachRampDistanceM) brings speed down before a
// known DISTANCE stop point, but there is no post-stop overshoot
// correction -- callers should not assume the true stop point is hit exactly.
bool follow_tape(LineFollowerContext *ctx, Direction dir, float speed_mps,
                  StopCondition stop_type, float stop_value, float timeout_s,
                  TapeFollowMode mode = TapeFollowMode::SINGLE_SENSOR);

#endif
