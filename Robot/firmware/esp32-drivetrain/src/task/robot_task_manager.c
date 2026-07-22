/** @file robot_task_manager.c
 *  @brief Coordinates tape, picking, and building task requests.
 */
#include "task/robot_task_manager.h"

#include <stddef.h>
#include <string.h>

/**
 * @brief Reports whether a phase has a task in progress.
 * @param phase Routine phase to inspect.
 * @return true when the phase is executable.
 */
static bool phase_is_active(RobotTaskPhase phase) {
    return phase >= ROBOT_TASK_PHASE_FOLLOW_TO_PIECES &&
           phase <= ROBOT_TASK_PHASE_BUILD_TOWER;
}

/**
 * @brief Builds and starts the request for the selected routine phase.
 * @param manager Manager containing routine parameters and executor state.
 * @param phase Phase to start.
 * @return true when the phase task was started.
 */
static bool start_phase(RobotTaskManager *manager, RobotTaskPhase phase) {
    TaskRequest request = {0};

    switch (phase) {
        case ROBOT_TASK_PHASE_FOLLOW_TO_PIECES:
            request.type = TASK_TYPE_TAPE_FOLLOWING;
            request.params.tape_following =
                manager->request.tape_to_pieces;
            break;
        case ROBOT_TASK_PHASE_PICK_UP_PIECES:
            request.type = TASK_TYPE_TOWER_PICKING;
            break;
        case ROBOT_TASK_PHASE_FOLLOW_TO_BASE:
            request.type = TASK_TYPE_TAPE_FOLLOWING;
            request.params.tape_following = manager->request.tape_to_base;
            break;
        case ROBOT_TASK_PHASE_BUILD_TOWER:
            request.type = TASK_TYPE_TOWER_BUILDING;
            break;
        default:
            return false;
    }

    if (!task_executor_start(&manager->executor, &request)) return false;

    manager->phase = phase;
    return true;
}

/**
 * @brief Starts the phase following a completed task.
 * @param manager Manager containing the completed phase.
 * @return true when the routine completed or its next phase started.
 */
static bool advance_phase(RobotTaskManager *manager) {
    switch (manager->phase) {
        case ROBOT_TASK_PHASE_FOLLOW_TO_PIECES:
            return start_phase(manager, ROBOT_TASK_PHASE_PICK_UP_PIECES);
        case ROBOT_TASK_PHASE_PICK_UP_PIECES:
            return start_phase(manager, ROBOT_TASK_PHASE_FOLLOW_TO_BASE);
        case ROBOT_TASK_PHASE_FOLLOW_TO_BASE:
            return start_phase(manager, ROBOT_TASK_PHASE_BUILD_TOWER);
        case ROBOT_TASK_PHASE_BUILD_TOWER:
            manager->phase = ROBOT_TASK_PHASE_COMPLETED;
            return true;
        default:
            return false;
    }
}

/**
 * @brief Resets a robot task manager.
 * @param manager Manager to initialize.
 */
void robot_task_manager_init(RobotTaskManager *manager) {
    if (manager == NULL) return;

    memset(manager, 0, sizeof(*manager));
    task_executor_init(&manager->executor);
    manager->phase = ROBOT_TASK_PHASE_IDLE;
}

/**
 * @brief Starts the complete tower routine.
 * @param manager Manager that will coordinate the routine.
 * @param request Routine parameters to copy.
 * @return true when the first phase was started.
 */
bool robot_task_manager_start(RobotTaskManager *manager,
                              const RobotTaskRoutineRequest *request) {
    if (manager == NULL || request == NULL ||
        phase_is_active(manager->phase)) {
        return false;
    }

    const TaskRequest tape_to_pieces = {
        .type = TASK_TYPE_TAPE_FOLLOWING,
        .params.tape_following = request->tape_to_pieces,
    };
    const TaskRequest tape_to_base = {
        .type = TASK_TYPE_TAPE_FOLLOWING,
        .params.tape_following = request->tape_to_base,
    };
    if (!task_request_is_valid(&tape_to_pieces) ||
        !task_request_is_valid(&tape_to_base)) {
        return false;
    }

    manager->request = *request;
    if (!start_phase(manager, ROBOT_TASK_PHASE_FOLLOW_TO_PIECES)) {
        manager->phase = ROBOT_TASK_PHASE_FAULTED;
        return false;
    }

    return true;
}

/**
 * @brief Updates the current task and starts the next completed phase.
 * @param manager Manager to update.
 */
void robot_task_manager_update(RobotTaskManager *manager) {
    if (manager == NULL || !phase_is_active(manager->phase)) return;

    task_executor_update(&manager->executor);

    const Task *task = task_executor_get_task(&manager->executor);
    if (task == NULL) {
        manager->phase = ROBOT_TASK_PHASE_FAULTED;
        return;
    }

    switch (task->state) {
        case TASK_STATE_ACTIVE:
            return;
        case TASK_STATE_COMPLETED:
            if (!advance_phase(manager)) {
                manager->phase = ROBOT_TASK_PHASE_FAULTED;
            }
            return;
        case TASK_STATE_CANCELLED:
            manager->phase = ROBOT_TASK_PHASE_CANCELLED;
            return;
        case TASK_STATE_FAULTED:
        default:
            manager->phase = ROBOT_TASK_PHASE_FAULTED;
            return;
    }
}

/**
 * @brief Cancels the active routine.
 * @param manager Manager containing the active routine.
 * @return true when the routine was cancelled.
 */
bool robot_task_manager_cancel(RobotTaskManager *manager) {
    if (manager == NULL || !phase_is_active(manager->phase) ||
        !task_executor_cancel(&manager->executor)) {
        return false;
    }

    manager->phase = ROBOT_TASK_PHASE_CANCELLED;
    return true;
}

/**
 * @brief Gets the current routine phase.
 * @param manager Manager to query.
 * @return Current phase, or ROBOT_TASK_PHASE_FAULTED for NULL.
 */
RobotTaskPhase robot_task_manager_get_phase(
    const RobotTaskManager *manager) {
    return manager == NULL ? ROBOT_TASK_PHASE_FAULTED : manager->phase;
}

/**
 * @brief Gets the task executing the current phase.
 * @param manager Manager to query.
 * @return Current task, or NULL when none has been started.
 */
const Task *robot_task_manager_get_task(const RobotTaskManager *manager) {
    if (manager == NULL) return NULL;

    return task_executor_get_task(&manager->executor);
}
