#ifndef LINE_FOLLOWER_HPP
#define LINE_FOLLOWER_HPP

#include "control/drivetrain/drivetrain.h"
#include "control/odometry/pose_service.h"
#include "drivers/tape_sensor/tape_sensor_driver.h"

enum class Direction { PX, MX, PY };

// LATERAL_ONE stops centered on the tape strip; LATERAL_TWO stops centered
// in the gap between two strips. Only valid for dir == PX (side sensor).
enum class StopCondition { LATERAL_ONE, LATERAL_TWO, DISTANCE, TIME_ONLY };

// Caller owns/initializes drivetrain, sensors, and pose_service once and
// reuses this context across calls. pose_service is borrowed: it's shared,
// continuously-updated state, not reset per maneuver. follow_tape() drives
// it each control cycle so pose and arm_uart keep advancing for the whole
// (blocking) duration of the maneuver, not just while main's loop() runs.
struct LineFollowerContext {
    Drivetrain *drivetrain;
    TapeSensor *sensors[3];  // front, back, side
    PoseService *pose_service;
};

// Runs tape-following until `stop_type` is satisfied or `timeout_s` elapses.
// Blocks the calling task for the duration of the maneuver. Returns true if
// the stop condition was reached, false on timeout, lost tape, or error.
//
// For DISTANCE and LATERAL_ONE/TWO stops, any overshoot past the true stop
// point (the requested distance, or the tape/gap center) is corrected
// internally with a low-speed reverse crawl before returning -- callers
// don't need to account for it.
bool follow_tape(LineFollowerContext *ctx, Direction dir, float speed_mps,
                  StopCondition stop_type, float stop_value, float timeout_s);

#endif
