/** @file task_runner.h
 *  @brief Task-specific step runner interfaces used by the executor.
 */
#pragma once

#include <stdint.h>

#include <robot_common/task/task.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Runs one update of a tower task step.
 * @param type Tower task workflow being executed.
 * @param step Current tower step index.
 * @return Current execution result of the step.
 */
TaskStepResult tower_task_run_step(TaskType type, uint16_t step);

/**
 * @brief Runs one update of a tape-following step.
 * @param step Current tape-following step index.
 * @param params Immutable tape-following parameters.
 * @return Current execution result of the step.
 */
TaskStepResult tape_task_run_step(uint16_t step,
                                  const TapeFollowingTaskParams *params);

#ifdef __cplusplus
}
#endif
