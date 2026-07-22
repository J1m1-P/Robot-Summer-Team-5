#include "config/communication/task_link_config.h"

const ArmTaskClientConfig ARM_TASK_CLIENT_CONFIG = {
    .heartbeat_interval_ms = 250U,
    .link_timeout_ms = 1000U,
    .command_retry_ms = 200U,
};

const TaskCoordinatorConfig TASK_COORDINATOR_CONFIG = {
    .step_timeout_ms = 30000U,
};
