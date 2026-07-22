/** @file task_action_executor.h
 *  @brief Minimal interface implemented by local and remote action managers.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/task/task.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TaskStepStatus status;
    TaskFailure failure;
} TaskActionResult;

typedef bool (*TaskActionStart)(void *context,
                                const TaskStepCommand *command,
                                uint32_t now_ms);
typedef TaskActionResult (*TaskActionUpdate)(void *context, uint32_t now_ms);
typedef void (*TaskActionCancel)(void *context, uint32_t now_ms);

typedef struct {
    void *context;
    TaskActionStart start;
    TaskActionUpdate update;
    TaskActionCancel cancel;
} TaskActionExecutor;

#ifdef __cplusplus
}
#endif
