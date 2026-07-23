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
    TASK_OWNER_TOP,
    TASK_OWNER_COUNT,
} TaskOwner;

/** Workflows currently supported by the coordinator. */
typedef enum {
    TASK_TYPE_TAPE_FOLLOWING = 0,
    TASK_TYPE_TOWER_PICKING,
    TASK_TYPE_TOWER_BUILDING,
    TASK_TYPE_TELETUBBY_SCAN,
    TASK_TYPE_COUNT,
} TaskType;

/** Physical actions assigned to one subsystem manager. */
typedef enum {
    TASK_ACTION_FOLLOW_TAPE = 0,
    TASK_ACTION_ALIGN_TO_PIECES = 1,
    TASK_ACTION_PICK_UP_BLOCK = 2,
    TASK_ACTION_ALIGN_TO_TAPE = 3,
    TASK_ACTION_BUILD_TOWER = 5,
    TASK_ACTION_SCAN_TELETUBBIES = 6,
    TASK_ACTION_FOLLOW_PIECES_TAPE = 7,
    TASK_ACTION_FOLLOW_TASK_TAPE = 8,
    TASK_ACTION_POSITION_TOWER_X = 9,
    TASK_ACTION_OPEN_TOWER_CLAWS = 10,
    TASK_ACTION_TOWER_FACE_DOWN = 11,
    TASK_ACTION_LOWER_TOWER = 12,
    TASK_ACTION_CLOSE_TOWER_CLAWS = 13,
    TASK_ACTION_RAISE_TOWER = 14,
    TASK_ACTION_TOWER_FACE_FRONT = 15,
    TASK_ACTION_BACK_OFF_PIECES = 16,
    TASK_ACTION_COUNT = 17,
} TaskAction;

#define TASK_ACTION_BIT(action) (UINT32_C(1) << (uint32_t)(action))
#define TASK_MAX_STEPS 16U

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
    TASK_FAILURE_NOT_IMPLEMENTED,
    TASK_FAILURE_SAFE_STATE_FAILED,
    TASK_FAILURE_EXECUTOR_UNAVAILABLE,
    TASK_FAILURE_TARGET_NOT_FOUND,
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

/** Type-specific request parameters. */
typedef union {
    TapeFollowingTaskParams tape_following;
} TaskParams;

/** Tuneable values attached to one workflow step.
 *  amount is radians or metres according to the action; speed is rad/s or
 *  m/s; settle_ms is the post-command mechanism settling time. */
typedef struct {
    float amount;
    float speed;
    uint32_t settle_ms;
} TaskStepParameters;

/** Minimal immutable request submitted to the coordinator. */
typedef struct {
    TaskType type;
    TaskParams params;
    TaskStepParameters step_parameters[TASK_MAX_STEPS];
    bool step_parameter_overrides[TASK_MAX_STEPS];
} TaskRequest;

/** Command passed from the coordinator to a local or remote manager. */
typedef struct {
    uint32_t execution_id;
    uint8_t step;
    TaskAction action;
    TapeFollowingTaskParams tape_following;
    TaskStepParameters parameters;
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
