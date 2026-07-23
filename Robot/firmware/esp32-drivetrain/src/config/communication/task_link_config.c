#include "config/communication/task_link_config.h"

const TaskLinkClientConfig TOP_TASK_CLIENT_CONFIG = {
    .heartbeat_interval_ms = 250U,
    .link_timeout_ms = 1000U,
    .command_retry_ms = 200U,
    .requester_endpoint = TASK_ENDPOINT_DRIVETRAIN,
    .executor_endpoint = TASK_ENDPOINT_TOP,
    .allowed_actions_mask =
        TASK_ACTION_BIT(TASK_ACTION_PICK_UP_BLOCK) |
        TASK_ACTION_BIT(TASK_ACTION_POSITION_TOWER_X) |
        TASK_ACTION_BIT(TASK_ACTION_OPEN_TOWER_CLAWS) |
        TASK_ACTION_BIT(TASK_ACTION_TOWER_FACE_DOWN) |
        TASK_ACTION_BIT(TASK_ACTION_LOWER_TOWER) |
        TASK_ACTION_BIT(TASK_ACTION_CLOSE_TOWER_CLAWS) |
        TASK_ACTION_BIT(TASK_ACTION_RAISE_TOWER) |
        TASK_ACTION_BIT(TASK_ACTION_TOWER_FACE_FRONT) |
        TASK_ACTION_BIT(TASK_ACTION_BUILD_TOWER) |
        TASK_ACTION_BIT(TASK_ACTION_SCAN_TELETUBBIES),
};

const TaskCoordinatorConfig TASK_COORDINATOR_CONFIG = {
    .step_timeout_ms = 30000U,
};
