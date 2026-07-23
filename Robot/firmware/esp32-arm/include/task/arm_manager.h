/**
 * @file arm_manager.h
 * @brief Owns and routes commands to independent arm action implementations.
 *
 * The manager selects either the tower pickup mechanism or tower-building
 * implementation, advances only that action, and exposes one executor to the
 * top dispatcher. Workflow ordering remains the drivetrain coordinator's job.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/task/task_action_executor.h>
#include "task/actions/build_tower_action.h"
#include "task/actions/pick_up_block_action.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TaskAction active_action; /**< Routed action, or TASK_ACTION_COUNT when idle. */
    PickUpBlockAction pick_up_block; /**< Tower pickup mechanism state. */
    BuildTowerAction build_tower; /**< Tower-building mechanism state. */
} ArmManager;

/** Initializes every arm action and leaves the manager idle. */
void arm_manager_init(ArmManager *manager);
/** Routes and starts one supported arm command when no action is running. */
bool arm_manager_start(ArmManager *manager, const TaskStepCommand *command,
                       uint32_t now_ms);
/** Advances the active arm action by one nonblocking loop cycle. */
void arm_manager_update(ArmManager *manager, uint32_t now_ms);
/** Stops the active arm action and returns whether cancellation was accepted. */
bool arm_manager_cancel(ArmManager *manager);
/** Test/harness hook that forces the active arm action to succeed. */
bool arm_manager_report_succeeded(ArmManager *manager);
/** Test/harness hook that fails the active action with a non-NONE reason. */
bool arm_manager_report_failed(ArmManager *manager, TaskFailure failure);
/** Returns the active action state and optionally copies its failure reason. */
TaskStepStatus arm_manager_get_status(const ArmManager *manager,
                                      TaskFailure *failure_out);
/** Wraps this manager in the generic TaskActionExecutor interface. */
TaskActionExecutor arm_manager_executor(ArmManager *manager);

#ifdef __cplusplus
}
#endif
