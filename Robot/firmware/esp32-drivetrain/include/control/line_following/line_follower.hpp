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

// Selects which module detects a marker while another module follows tape.
enum class TapeMarkerSensor { AUTO, FRONT, BACK, SIDE };

// RISE_ONE stops on one detected tape edge; RISE_TWO waits for two tape edges
// with a gap between them. ALL_CHANNELS_ON stops when all four channels of the
// sensor used for the current travel direction are on tape. AUTO marker
// selection uses side for PX travel, back for PY travel, and no marker module
// for MX travel.
enum class StopCondition {
    RISE_ONE,
    RISE_TWO,
    ALL_CHANNELS_ON,
    DISTANCE,
    TIME_ONLY,
};

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
                  TapeFollowMode mode = TapeFollowMode::SINGLE_SENSOR,
                  TapeMarkerSensor marker_sensor = TapeMarkerSensor::AUTO);

// PX tape-follow variant that returns the world heading of a regression line
// fitted through five discrete pose samples from 13 cm to 3 cm before the
// all-channels endpoint. The caller applies that heading after the stop.
bool follow_tape_until_all_channels_average_heading(
    LineFollowerContext *ctx, float speed_mps, float timeout_s,
    float *average_heading_rad_out);

#endif
