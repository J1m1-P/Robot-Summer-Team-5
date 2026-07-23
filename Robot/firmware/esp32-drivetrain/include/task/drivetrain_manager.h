/**
 * @file drivetrain_manager.h
 * @brief Owns shared drivetrain task hardware and routes local action commands.
 *
 * The manager initializes the tape sensors and controller once, stores each
 * independent drivetrain action, and exposes them as one TaskActionExecutor.
 * It executes commands but never decides workflow order.
 */
#pragma once

#include "control/drivetrain/drivetrain.h"
#include "control/tape_following/tape_follower.h"
#include "drivers/tape_sensor/tape_sensor_driver.h"
#include "task/actions/follow_tape_action.h"
#include "task/actions/tape_alignment_action.h"
#include <robot_common/task/task_action_executor.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Drivetrain *drivetrain; /**< Low-level drive controller used by actions. */
    TaskAction active_action; /**< Routed action, or TASK_ACTION_COUNT when idle. */

    // Shared tape hardware used for route following and the two tower-picking
    // alignment actions.
    TapeSensorMux tape_mux; /**< Multiplexer shared by all tape modules. */
    TapeSensor tape_sensor_front; /**< Front tape module runtime. */
    TapeSensor tape_sensor_back; /**< Rear tape module runtime. */
    // Sampled alongside front/back only because tape_sensor_driver_read_all
    // always reads all three modules; not otherwise used by tape following.
    TapeSensor tape_sensor_left; /**< Left alignment/detection module runtime. */
    bool tape_hardware_ready; /**< False makes every drivetrain action reject. */
    TapeFollower tape_follower; /**< Shared line estimator/controller state. */
    FollowTapeAction follow_tape; /**< Normal route-following implementation. */
    TapeAlignmentAction tape_alignment; /**< Picking alignment implementation. */
} DrivetrainManager;

// Initializes tape-following hardware once and prepares an idle manager.
// Returns the first hardware error (if any) but always leaves the manager in
// a usable state: on failure, tape_hardware_ready is false and
// TASK_ACTION_FOLLOW_TAPE will be rejected rather than silently hanging.
esp_err_t drivetrain_manager_init(DrivetrainManager *manager,
                                  Drivetrain *drivetrain,
                                  const TapeSensorMuxConfig *tape_mux_config,
                                  const TapeSensorDriverConfig *front_sensor_config,
                                  const TapeSensorDriverConfig *back_sensor_config,
                                  const TapeSensorDriverConfig *left_sensor_config,
                                  const TapeFollowerConfig *tape_follower_config);
TaskActionExecutor drivetrain_manager_executor(DrivetrainManager *manager);
/** Forces the currently active action to report successful completion. */
bool drivetrain_manager_report_succeeded(DrivetrainManager *manager);
/** Forces the active action to report the supplied non-NONE failure. */
bool drivetrain_manager_report_failed(DrivetrainManager *manager,
                                      TaskFailure failure);

#ifdef __cplusplus
}
#endif
