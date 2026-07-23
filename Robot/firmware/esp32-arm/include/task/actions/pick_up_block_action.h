/** @file pick_up_block_action.h
 *  @brief Stateful implementation boundary for TASK_ACTION_PICK_UP_BLOCK.
 */
#pragma once

#include <robot_common/task/task_action_executor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TaskActionResult result;
} PickUpBlockAction;

void pick_up_block_action_init(PickUpBlockAction *action);
bool pick_up_block_action_start(PickUpBlockAction *action,
                                const TaskStepCommand *command,
                                uint32_t now_ms);
void pick_up_block_action_update(PickUpBlockAction *action, uint32_t now_ms);
bool pick_up_block_action_cancel(PickUpBlockAction *action);
bool pick_up_block_action_report_succeeded(PickUpBlockAction *action);
bool pick_up_block_action_report_failed(PickUpBlockAction *action,
                                        TaskFailure failure);

#ifdef __cplusplus
}
#endif
