/**
 * @file production_task_config.c
 * @brief Selects and parameterizes the task automatically run after boot.
 *
 * The current production policy waits briefly and then requests forward tape
 * following. Changing this object selects inputs, not workflow step logic.
 */
#include "config/task/production_task_config.h"

// Placement delay measured from completion of drivetrain setup.
const uint32_t PRODUCTION_TASK_START_DELAY_MS = 3000U;

// Conservative placeholder -- verify direction and tune speed/distance on
// the assembled robot before competition use.
const TaskRequest PRODUCTION_TASK_REQUEST = {
    .type = TASK_TYPE_TAPE_FOLLOWING,
    .step_parameters = {
        {
            .amount = PRODUCTION_TAPE_FOLLOW_DISTANCE_M,
            .speed = 0.1f,
        },
    },
    .step_parameter_override_mask = UINT16_C(1),
};
