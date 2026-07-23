/* Defines the fixed workflow the production coordinator auto-starts at boot. */
#pragma once

#include <stdint.h>

#include <robot_common/task/task.h>

#ifdef __cplusplus
extern "C" {
#endif

// Delay after setup() completes before task_coordinator_start() is called,
// giving time to place the robot on the field before it moves.
extern const uint32_t PRODUCTION_TASK_START_DELAY_MS;

// Initial tape-following travel target. Tune this value on the assembled robot.
#define PRODUCTION_TAPE_FOLLOW_DISTANCE_M 3.0f

// Fixed request auto-started once at boot. Placeholder speed/distance --
// tune on the assembled robot before competition use.
extern const TaskRequest PRODUCTION_TASK_REQUEST;

#ifdef __cplusplus
}
#endif
