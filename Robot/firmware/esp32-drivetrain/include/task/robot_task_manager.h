/** @file robot_task_manager.h
 *  @brief Coordinates the robot's multi-task tower routine.
 */
#pragma once

#include <stdbool.h>

#include "task/task_executor.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Current phase of the coordinated tower routine. */
typedef enum {
    ROBOT_TASK_PHASE_IDLE,
    ROBOT_TASK_PHASE_FOLLOW_TO_PIECES,
    ROBOT_TASK_PHASE_PICK_UP_PIECES,
    ROBOT_TASK_PHASE_FOLLOW_TO_BASE,
    ROBOT_TASK_PHASE_BUILD_TOWER,
    ROBOT_TASK_PHASE_COMPLETED,
    ROBOT_TASK_PHASE_CANCELLED,
    ROBOT_TASK_PHASE_FAULTED,
} RobotTaskPhase;

/** @brief Parameters for the complete tower routine. */
typedef struct {
    TapeFollowingTaskParams tape_to_pieces; /**< First tape segment. */
    TapeFollowingTaskParams tape_to_base;   /**< Second tape segment. */
} RobotTaskRoutineRequest;

/** @brief Owns routine state and its single-task executor. */
typedef struct {
    TaskExecutor executor;            /**< Executes the current phase. */
    RobotTaskRoutineRequest request;  /**< Copied routine parameters. */
    RobotTaskPhase phase;             /**< Current routine phase. */
} RobotTaskManager;

/**
 * @brief Resets a robot task manager.
 * @param manager Manager to initialize.
 */
void robot_task_manager_init(RobotTaskManager *manager);

/**
 * @brief Starts the complete tower routine.
 * @param manager Manager that will coordinate the routine.
 * @param request Routine parameters to copy.
 * @return true when the first phase was started.
 */
bool robot_task_manager_start(RobotTaskManager *manager,
                              const RobotTaskRoutineRequest *request);

/**
 * @brief Updates the current task and starts the next completed phase.
 * @param manager Manager to update.
 */
void robot_task_manager_update(RobotTaskManager *manager);

/**
 * @brief Cancels the active routine.
 * @param manager Manager containing the active routine.
 * @return true when the routine was cancelled.
 */
bool robot_task_manager_cancel(RobotTaskManager *manager);

/**
 * @brief Gets the current routine phase.
 * @param manager Manager to query.
 * @return Current phase, or ROBOT_TASK_PHASE_FAULTED for NULL.
 */
RobotTaskPhase robot_task_manager_get_phase(
    const RobotTaskManager *manager);

/**
 * @brief Gets the task executing the current phase.
 * @param manager Manager to query.
 * @return Current task, or NULL when none has been started.
 */
const Task *robot_task_manager_get_task(const RobotTaskManager *manager);

#ifdef __cplusplus
}
#endif
