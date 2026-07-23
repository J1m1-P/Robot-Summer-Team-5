#include "config/task/production_task_config.h"

const uint32_t PRODUCTION_TASK_START_DELAY_MS = 3000U;

// Conservative placeholder -- verify direction and tune speed/distance on
// the assembled robot before competition use.
const TaskRequest PRODUCTION_TASK_REQUEST = {
    .type = TASK_TYPE_TAPE_FOLLOWING,
    .params.tape_following = {
        .direction = TAPE_DIRECTION_FORWARD,
        .speed_mps = 0.1f,
        .distance_m = 0.3f,
    },
};
