/**
 * @file task_link_config.h
 * @brief Declares drivetrain-side remote-task timing and action policy.
 *
 * These constants configure the client that sends top-owned actions to the arm
 * ESP32 and the coordinator timeout applied to every workflow step.
 */
#pragma once

#include <robot_common/task/task_link_client.h>
#include "task/task_coordinator.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Reliable drivetrain-to-top task-link client configuration. */
extern const TaskLinkClientConfig TOP_TASK_CLIENT_CONFIG;
/** Maximum runtime policy used by the authoritative task coordinator. */
extern const TaskCoordinatorConfig TASK_COORDINATOR_CONFIG;

#ifdef __cplusplus
}
#endif
