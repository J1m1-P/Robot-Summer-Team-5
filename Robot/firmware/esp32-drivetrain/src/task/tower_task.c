/** @file tower_task.c
 *  @brief Implements tower-picking and tower-building step execution.
 */
#include "task/task_runner.h"

/** @brief Aligns the drivetrain with loose tower pieces. */
static TaskStepResult align_to_pieces(void) {
    /* TODO: Start/update drivetrain alignment with the pieces. */
    return TASK_STEP_RUNNING;
}

/** @brief Picks up a tower piece with the arm. */
static TaskStepResult pick_up_block(void) {
    /* TODO: Request the arm action and read its reported result. */
    return TASK_STEP_RUNNING;
}

/** @brief Aligns the drivetrain with the tower base. */
static TaskStepResult align_to_base(void) {
    /* TODO: Start/update drivetrain alignment with the base. */
    return TASK_STEP_RUNNING;
}

/** @brief Builds the tower with the arm. */
static TaskStepResult build_tower(void) {
    /* TODO: Request the arm action and read its reported result. */
    return TASK_STEP_RUNNING;
}

/** @brief Aligns the drivetrain with tape after a tower action. */
static TaskStepResult align_to_tape(void) {
    /* TODO: Start/update drivetrain alignment with the tape. */
    return TASK_STEP_RUNNING;
}

/**
 * @brief Runs one tower-picking step update.
 * @param step Current tower-picking step index.
 * @return Current execution result of the step.
 */
static TaskStepResult run_picking_step(uint16_t step) {
    switch (step) {
        case TOWER_PICKING_STEP_ALIGN_TO_PIECES:
            return align_to_pieces();
        case TOWER_PICKING_STEP_PICK_UP_BLOCK:
            return pick_up_block();
        case TOWER_PICKING_STEP_ALIGN_TO_TAPE:
            return align_to_tape();
        default:
            return TASK_STEP_FAILED;
    }
}

/**
 * @brief Runs one tower-building step update.
 * @param step Current tower-building step index.
 * @return Current execution result of the step.
 */
static TaskStepResult run_building_step(uint16_t step) {
    switch (step) {
        case TOWER_BUILDING_STEP_ALIGN_TO_BASE:
            return align_to_base();
        case TOWER_BUILDING_STEP_BUILD_TOWER:
            return build_tower();
        case TOWER_BUILDING_STEP_ALIGN_TO_TAPE:
            return align_to_tape();
        default:
            return TASK_STEP_FAILED;
    }
}

/**
 * @brief Runs one update of a tower task step.
 * @param type Tower task workflow being executed.
 * @param step Current tower step index.
 * @return Current execution result of the step.
 */
TaskStepResult tower_task_run_step(TaskType type, uint16_t step) {
    switch (type) {
        case TASK_TYPE_TOWER_PICKING:
            return run_picking_step(step);
        case TASK_TYPE_TOWER_BUILDING:
            return run_building_step(step);
        default:
            return TASK_STEP_FAILED;
    }
}
