/** @file task.h
 *  @brief Shared task requests, workflow definitions, and runtime state.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Controller that physically executes a workflow action. */
typedef enum {
    TASK_OWNER_DRIVETRAIN = 0,
    TASK_OWNER_ARM,
    TASK_OWNER_COUNT,
} TaskOwner;

/** Workflows currently supported by the coordinator. */
typedef enum {
    TASK_TYPE_TAPE_FOLLOWING = 0,
    TASK_TYPE_TOWER_PICKING,
    TASK_TYPE_TOWER_BUILDING,
    TASK_TYPE_TOWER_ROUTINE,
    TASK_TYPE_COUNT,
} TaskType;

/** Physical actions assigned to one subsystem manager. */
typedef enum {
    TASK_ACTION_FOLLOW_TAPE = 0,
    TASK_ACTION_ALIGN_TO_PIECES,
    TASK_ACTION_PICK_UP_BLOCK,
    TASK_ACTION_ALIGN_TO_TAPE,
    TASK_ACTION_ALIGN_TO_BASE,
    TASK_ACTION_BUILD_TOWER,
    TASK_ACTION_COUNT,
} TaskAction;

/** Authoritative lifecycle state of one task execution. */
typedef enum {
    TASK_STATUS_IDLE = 0,
    TASK_STATUS_RUNNING,
    TASK_STATUS_SUCCEEDED,
    TASK_STATUS_CANCELLED,
    TASK_STATUS_FAILED,
} TaskStatus;

/** Lifecycle state of the task's current action. */
typedef enum {
    TASK_STEP_NOT_STARTED = 0,
    TASK_STEP_RUNNING,
    TASK_STEP_SUCCEEDED,
    TASK_STEP_CANCELLED,
    TASK_STEP_FAILED,
} TaskStepStatus;

/** Stable reason codes used for task behavior and wire status. */
typedef enum {
    TASK_FAILURE_NONE = 0,
    TASK_FAILURE_BUSY,
    TASK_FAILURE_INVALID_STEP,
    TASK_FAILURE_STEP_REJECTED,
    TASK_FAILURE_STEP_FAILED,
    TASK_FAILURE_STEP_TIMEOUT,
    TASK_FAILURE_LINK_TIMEOUT,
    TASK_FAILURE_PEER_RESET,
    TASK_FAILURE_STALE_MESSAGE,
    TASK_FAILURE_PROTOCOL,
    TASK_FAILURE_COUNT,
} TaskFailure;

/** Direction of travel during tape following. */
typedef enum {
    TAPE_DIRECTION_FORWARD = 0,
    TAPE_DIRECTION_REVERSE,
} TapeDirection;

/** Immutable inputs for one tape-following action. */
typedef struct {
    TapeDirection direction;
    float speed_mps;
    float distance_m;
} TapeFollowingTaskParams;

/** Immutable inputs for the complete collect-and-build routine. */
typedef struct {
    TapeFollowingTaskParams tape_to_pieces;
    TapeFollowingTaskParams tape_to_base;
} TowerRoutineTaskParams;

/** Type-specific request parameters. */
typedef union {
    TapeFollowingTaskParams tape_following;
    TowerRoutineTaskParams tower_routine;
} TaskParams;

/** Minimal immutable request submitted to the coordinator. */
typedef struct {
    TaskType type;
    TaskParams params;
} TaskRequest;

/** Command passed from the coordinator to a local or remote manager. */
typedef struct {
    uint32_t execution_id;
    uint8_t step;
    TaskAction action;
    TapeFollowingTaskParams tape_following;
} TaskStepCommand;

/** The coordinator's single authoritative mutable task record. */
typedef struct {
    uint32_t execution_id;
    TaskRequest request;
    uint8_t current_step;
    TaskStatus status;
    TaskStepStatus step_status;
    TaskFailure failure;
} TaskRuntime;

bool task_request_is_valid(const TaskRequest *request);
bool task_action_is_valid(TaskAction action);
bool task_step_status_is_terminal(TaskStepStatus status);

#ifdef __cplusplus
}
#endif
