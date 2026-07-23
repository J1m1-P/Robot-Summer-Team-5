/**
 * @file production_task_config.h
 * @brief Declares the fixed task request automatically started in production.
 *
 * This configuration chooses startup timing and request parameters only; the
 * authoritative workflow steps remain in task_coordinator.c.
 */
#pragma once

#include <stdint.h>

#include <robot_common/task/task.h>

#ifdef __cplusplus
extern "C" {
#endif

// Delay after setup() completes before task_coordinator_start() is called,
// giving time to place the robot on the field before it moves.
extern const uint32_t PRODUCTION_TASK_START_DELAY_MS;

// Initial tape-following travel target in metres; tune on assembled hardware.
#define PRODUCTION_TAPE_FOLLOW_DISTANCE_M 3.0f

// Fixed request auto-started once at boot. Placeholder speed/distance --
// tune on the assembled robot before competition use.
extern const TaskRequest PRODUCTION_TASK_REQUEST;

#ifdef __cplusplus
}
#endif
