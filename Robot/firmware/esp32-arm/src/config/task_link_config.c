#include "config/task_link_config.h"

const TaskLinkServerConfig TOP_TASK_SERVER_CONFIG = {
    .heartbeat_interval_ms = 250U,
    .link_timeout_ms = 1000U,
    .status_interval_ms = 250U,
    .requester_endpoint = TASK_ENDPOINT_DRIVETRAIN,
    .executor_endpoint = TASK_ENDPOINT_TOP,
};

const TaskLinkClientConfig PI_SCAN_CLIENT_CONFIG = {
    .heartbeat_interval_ms = 250U,
    .link_timeout_ms = 2000U,
    .command_retry_ms = 250U,
    .requester_endpoint = TASK_ENDPOINT_TOP,
    .executor_endpoint = TASK_ENDPOINT_PI,
    .allowed_actions_mask = TASK_ACTION_BIT(TASK_ACTION_SCAN_TELETUBBIES),
};
