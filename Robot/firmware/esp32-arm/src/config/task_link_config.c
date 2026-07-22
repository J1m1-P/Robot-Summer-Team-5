#include "config/task_link_config.h"

const ArmTaskServerConfig ARM_TASK_SERVER_CONFIG = {
    .heartbeat_interval_ms = 250U,
    .link_timeout_ms = 1000U,
    .status_interval_ms = 250U,
};
