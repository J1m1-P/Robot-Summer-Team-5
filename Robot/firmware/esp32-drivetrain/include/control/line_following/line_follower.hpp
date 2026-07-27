#ifndef LINE_FOLLOWER_HPP
#define LINE_FOLLOWER_HPP

#include "control/drivetrain/drivetrain.h"
#include "control/odometry/pose_tracker.h"
#include "drivers/tape_sensor/tape_sensor_driver.h"

enum class Direction { PX, MX, PY };

// LATERAL_ONE stops centered on the tape strip; LATERAL_TWO stops centered
// in the gap between two strips. Only valid for dir == PX (side sensor).
enum class StopCondition { LATERAL_ONE, LATERAL_TWO, DISTANCE, TIME_ONLY };

// Caller owns/initializes drivetrain, sensors, and pose_tracker once and
// reuses this context across calls. pose_tracker is borrowed: it's shared,
// continuously-updated state, not reset per maneuver.
struct LineFollowerContext {
    Drivetrain *drivetrain;
    TapeSensor *sensors[3];  // front, back, side
    PoseTracker *pose_tracker;
};

// Runs tape-following until `stop_type` is satisfied or `timeout_s` elapses.
// Blocks the calling task for the duration of the maneuver. Returns true if
// the stop condition was reached, false on timeout, lost tape, or error.
bool follow_tape(LineFollowerContext *ctx, Direction dir, float speed_mps,
                  StopCondition stop_type, float stop_value, float timeout_s);

#endif
