/**
 * @file task_link_config.c
 * @brief Defines arm task-server and Raspberry Pi task-client reliability.
 *
 * Endpoint identities and timing values keep the two serial links independent.
 * The Pi client allowlist ensures only the scan action crosses that boundary.
 */
#include "config/task_link_config.h"

// Receives all top-owned commands issued by the drivetrain endpoint.
const TaskLinkServerConfig TOP_TASK_SERVER_CONFIG = {
    .heartbeat_interval_ms = 250U,
    .link_timeout_ms = 1000U,
    .status_interval_ms = 250U,
    .requester_endpoint = TASK_ENDPOINT_DRIVETRAIN,
    .executor_endpoint = TASK_ENDPOINT_TOP,
};

// Forwards only SCAN_TELETUBBIES from the arm endpoint to the Pi endpoint.
const TaskLinkClientConfig PI_SCAN_CLIENT_CONFIG = {
    .heartbeat_interval_ms = 250U,
    .link_timeout_ms = 2000U,
    .command_retry_ms = 250U,
    .requester_endpoint = TASK_ENDPOINT_TOP,
    .executor_endpoint = TASK_ENDPOINT_PI,
    .allowed_actions_mask = TASK_ACTION_BIT(TASK_ACTION_SCAN_TELETUBBIES),
};
