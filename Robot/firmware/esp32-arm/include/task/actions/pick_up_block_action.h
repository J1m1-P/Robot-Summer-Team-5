/**
 * @file pick_up_block_action.h
 * @brief Declares the tower-claw actions used by the picking workflow.
 *
 * One state object drives tower X/Z steppers and four servos for the discrete
 * pickup steps. Commands remain nonblocking: steppers complete when motion
 * stops and servo commands complete after their requested settling time.
 */
#pragma once

#include <robot_common/task/task_action_executor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TaskActionResult result; /**< Latest mechanism action state. */
    TaskAction active_action; /**< Discrete claw/stepper behavior being run. */
    TaskStepParameters parameters; /**< Distance and settling inputs. */
    uint32_t started_ms; /**< Start time used for servo settling. */
    bool hardware_ready; /**< True after every servo/stepper initializes. */
} PickUpBlockAction;

/** Initializes all tower pickup servos/steppers and leaves the action idle. */
void pick_up_block_action_init(PickUpBlockAction *action);
/** Applies one supported mechanism command and starts its completion tracking. */
bool pick_up_block_action_start(PickUpBlockAction *action,
                                const TaskStepCommand *command,
                                uint32_t now_ms);
/** Advances steppers and completes servo actions after their settling delay. */
void pick_up_block_action_update(PickUpBlockAction *action, uint32_t now_ms);
/** Stops both tower steppers and marks a running action cancelled. */
bool pick_up_block_action_cancel(PickUpBlockAction *action);
/** Test/harness hook that forces a running mechanism step to succeed. */
bool pick_up_block_action_report_succeeded(PickUpBlockAction *action);
/** Test/harness hook that fails a running mechanism step with a reason. */
bool pick_up_block_action_report_failed(PickUpBlockAction *action,
                                        TaskFailure failure);

#ifdef __cplusplus
}
#endif
