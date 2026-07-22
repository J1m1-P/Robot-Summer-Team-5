/** @file task.h
 *  @brief Shared task requests, runtime state, steps, and metadata.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Task lifecycle and execution results
 * -------------------------------------------------------------------------- */

/** @brief Subsystem responsible for executing a task step. */
typedef enum {
    TASK_OWNER_DRIVETRAIN,
    TASK_OWNER_ARM,
} TaskOwner;

/** @brief Supported single-task workflows. */
typedef enum {
    TASK_TYPE_TOWER_PICKING,
    TASK_TYPE_TOWER_BUILDING,
    TASK_TYPE_TAPE_FOLLOWING,
} TaskType;

/** @brief Runtime state of a task. */
typedef enum {
    TASK_STATE_ACTIVE,
    TASK_STATE_COMPLETED,
    TASK_STATE_CANCELLED,
    TASK_STATE_FAULTED,
} TaskState;

/** @brief Current execution result of a task step. */
typedef enum {
    TASK_STEP_RUNNING,
    TASK_STEP_SUCCEEDED,
    TASK_STEP_FAILED,
} TaskStepResult;

/* --------------------------------------------------------------------------
 * Task requests and immutable parameters
 * -------------------------------------------------------------------------- */

/** @brief Travel direction for a tape-following task. */
typedef enum {
    TAPE_DIRECTION_FORWARD,
    TAPE_DIRECTION_REVERSE,
} TapeDirection;

/** @brief Inputs that configure a tape-following task. */
typedef struct {
    TapeDirection direction; /**< Direction of travel along the tape. */
    float speed_mps;         /**< Requested positive travel speed. */
    float distance_m;        /**< Requested positive travel distance. */
} TapeFollowingTaskParams;

/** @brief Parameters for the selected task type. */
typedef union {
    TapeFollowingTaskParams tape_following; /**< Tape-following inputs. */
} TaskParams;

/** @brief A task type paired with its immutable parameters. */
typedef struct {
    TaskType type;     /**< Workflow to execute. */
    TaskParams params; /**< Inputs for the selected workflow. */
} TaskRequest;

/* --------------------------------------------------------------------------
 * Mutable task runtime state
 * -------------------------------------------------------------------------- */

/** @brief Current state and routing information for one task. */
typedef struct {
    TaskRequest request;   /**< Immutable request being executed. */
    TaskOwner owner;       /**< Subsystem responsible for the current step. */
    uint16_t current_step; /**< Current workflow step index. */
    TaskState state;       /**< Current task lifecycle state. */
} Task;

/* --------------------------------------------------------------------------
 * Task-specific workflow steps
 * -------------------------------------------------------------------------- */

/** @brief Ordered steps for collecting tower pieces. */
typedef enum {
    TOWER_PICKING_STEP_ALIGN_TO_PIECES = 0,
    TOWER_PICKING_STEP_PICK_UP_BLOCK,
    TOWER_PICKING_STEP_ALIGN_TO_TAPE,
    TOWER_PICKING_STEP_COUNT
} TowerPickingStep;

/** @brief Ordered steps for building a tower. */
typedef enum {
    TOWER_BUILDING_STEP_ALIGN_TO_BASE = 0,
    TOWER_BUILDING_STEP_BUILD_TOWER,
    TOWER_BUILDING_STEP_ALIGN_TO_TAPE,
    TOWER_BUILDING_STEP_COUNT
} TowerBuildingStep;

/** @brief Ordered steps for following tape. */
typedef enum {
    TAPE_FOLLOWING_STEP_FOLLOW_TAPE = 0,
    TAPE_FOLLOWING_STEP_COUNT
} TapeFollowingStep;

/* --------------------------------------------------------------------------
 * Request validation and workflow metadata
 * -------------------------------------------------------------------------- */

/**
 * @brief Validates a task request and its type-specific parameters.
 * @param request Request to validate.
 * @return true when the request can be started.
 */
bool task_request_is_valid(const TaskRequest *request);

/**
 * @brief Gets the owner of a step for a supported task type.
 * @param type Task workflow containing the step.
 * @param step Workflow step index.
 * @param owner_out Receives the responsible subsystem.
 * @return true when the type and step are valid.
 */
bool task_get_step_owner(TaskType type, uint16_t step,
                         TaskOwner *owner_out);

/**
 * @brief Gets the number of steps for a supported task type.
 * @param type Task workflow to inspect.
 * @param count_out Receives the workflow step count.
 * @return true when the task type is supported.
 */
bool task_get_step_count(TaskType type, uint16_t *count_out);

#ifdef __cplusplus
}
#endif
