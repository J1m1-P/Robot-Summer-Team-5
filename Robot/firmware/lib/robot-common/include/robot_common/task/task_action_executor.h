/**
 * @file task_action_executor.h
 * @brief Defines the common start/update/cancel interface for task actions.
 *
 * The coordinator uses this interface without knowing whether an action runs
 * in a local manager or across a UART task link. Implementations must remain
 * nonblocking: start begins work, update polls it, and cancel stops it.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/task/task.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TaskStepStatus status; /**< Current or terminal action lifecycle state. */
    TaskFailure failure;   /**< Failure reason, or NONE for nonfailed states. */
} TaskActionResult;

/** Starts one command on the executor represented by context. */
typedef bool (*TaskActionStart)(void *context,
                                 const TaskStepCommand *command,
                                 uint32_t now_ms);
/** Advances or polls the active action and returns its latest result. */
typedef TaskActionResult (*TaskActionUpdate)(void *context, uint32_t now_ms);
/** Stops the active action and places its hardware in an inactive state. */
typedef void (*TaskActionCancel)(void *context, uint32_t now_ms);

typedef struct {
    void *context;          /**< Implementation-owned state passed to callbacks. */
    TaskActionStart start;  /**< Nonblocking action start callback. */
    TaskActionUpdate update; /**< Nonblocking action poll callback. */
    TaskActionCancel cancel; /**< Immediate cancellation callback. */
} TaskActionExecutor;

#ifdef __cplusplus
}
#endif
