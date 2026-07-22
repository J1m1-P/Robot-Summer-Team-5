/** @file tape_task.c
 *  @brief Implements tape-following task step execution.
 */
#include "task/task_runner.h"

#include <stddef.h>

/**
 * @brief Follows tape until the task destination is reached.
 * @param params Immutable tape-following parameters.
 * @return Current execution result of the step.
 */
static TaskStepResult follow_tape(const TapeFollowingTaskParams *params) {
    /* TODO: Update the tape follower and report its completion state. */
    (void)params;
    return TASK_STEP_RUNNING;
}

/**
 * @brief Runs one update of a tape-following step.
 * @param step Current tape-following step index.
 * @param params Immutable tape-following parameters.
 * @return Current execution result of the step.
 */
TaskStepResult tape_task_run_step(uint16_t step,
                                  const TapeFollowingTaskParams *params) {
    if (params == NULL) return TASK_STEP_FAILED;

    switch (step) {
        case TAPE_FOLLOWING_STEP_FOLLOW_TAPE:
            return follow_tape(params);
        default:
            return TASK_STEP_FAILED;
    }
}
