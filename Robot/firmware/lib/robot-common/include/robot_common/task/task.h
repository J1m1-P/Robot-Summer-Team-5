/**
 * @file task.h
 * @brief Defines the shared language used by every task-system endpoint.
 *
 * These enums and data structures describe task requests, individual action
 * commands, lifecycle results, and the coordinator's authoritative runtime.
 * Keep this file transport-independent so the same values work locally and
 * across the ESP32 and Raspberry Pi task links.
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
    TapeDirection direction; /**< Forward or reverse travel along the tape. */
    float speed_mps;         /**< Positive commanded travel speed. */
    float distance_m;        /**< Positive path length that completes the step. */
} TapeFollowingTaskParams;

/** Type-specific request parameters. */
typedef union {
    TapeFollowingTaskParams tape_following;
} TaskParams;

/** Tuneable values attached to one workflow step.
 *  amount is radians or metres according to the action; speed is rad/s or
 *  m/s; settle_ms is the post-command mechanism settling time. */
typedef struct {
    float amount;       /**< Action-specific angle or linear distance. */
    float speed;        /**< Action-specific angular or linear speed. */
    uint32_t settle_ms; /**< Time allowed for a mechanism to settle. */
} TaskStepParameters;

/** Minimal immutable request submitted to the coordinator. */
typedef struct {
    TaskType type; /**< Workflow selected by the caller. */
    TaskParams params; /**< Parameters used by the selected workflow type. */
    TaskStepParameters step_parameters[TASK_MAX_STEPS]; /**< Optional values by step. */
    bool step_parameter_overrides[TASK_MAX_STEPS]; /**< True when the matching value overrides the workflow default. */
} TaskRequest;

/** Command passed from the coordinator to a local or remote manager. */
typedef struct {
    uint32_t execution_id; /**< Identity shared by every step in one run. */
    uint8_t step;          /**< Zero-based workflow step index. */
    TaskAction action;     /**< Physical behavior the executor must run. */
    TapeFollowingTaskParams tape_following; /**< Tape-specific input when applicable. */
    TaskStepParameters parameters; /**< Generic action tuning values. */
} TaskStepCommand;

/** The coordinator's single authoritative mutable task record. */
typedef struct {
    uint32_t execution_id; /**< Nonzero identity for the active or last run. */
    TaskRequest request;   /**< Immutable request copied at task start. */
    uint8_t current_step;  /**< Zero-based authoritative step position. */
    TaskStatus status;     /**< Overall task lifecycle state. */
    TaskStepStatus step_status; /**< Lifecycle state of current_step. */
    TaskFailure failure;   /**< Stable reason when the task has failed. */
} TaskRuntime;

/** Validates a request and the parameters required by its selected type. */
bool task_request_is_valid(const TaskRequest *request);
/** Returns true only for defined, executable TaskAction values. */
bool task_action_is_valid(TaskAction action);
/** Reports whether a step result can no longer return to running. */
bool task_step_status_is_terminal(TaskStepStatus status);

#ifdef __cplusplus
}
#endif
