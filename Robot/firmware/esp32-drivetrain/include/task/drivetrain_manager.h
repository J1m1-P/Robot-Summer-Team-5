/** @file drivetrain_manager.h
 *  @brief Executes drivetrain-owned actions without sequencing tasks.
 */
#pragma once

#include "control/drivetrain/drivetrain.h"
#include "control/drivetrain/odometry.h"
#include "control/tape_following/tape_follower.h"
#include "drivers/tape_sensor/tape_sensor_driver.h"
#include "task/task_action_executor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    Drivetrain *drivetrain;
    TaskActionResult result;
    TaskAction active_action;

    // Tape-following hardware and behavior state. TASK_ACTION_FOLLOW_TAPE is
    // the only action currently driven end-to-end; ALIGN_* actions are
    // accepted (so workflow sequencing doesn't stall) but still only flip
    // logical status until their own physical behavior is implemented.
    TapeSensorMux tape_mux;
    TapeSensor tape_sensor_front;
    TapeSensor tape_sensor_back;
    // Sampled alongside front/back only because tape_sensor_driver_read_all
    // always reads all three modules; not otherwise used by tape following.
    TapeSensor tape_sensor_left;
    bool tape_hardware_ready;
    TapeFollower tape_follower;

    DrivetrainOdometry odometry;
    int32_t last_encoder_counts[DRIVETRAIN_MOTOR_MAX];
    uint32_t last_update_ms;
    float signed_travel_speed_mps;
    float target_distance_m;
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
bool drivetrain_manager_report_succeeded(DrivetrainManager *manager);
bool drivetrain_manager_report_failed(DrivetrainManager *manager,
                                      TaskFailure failure);

#ifdef __cplusplus
}
#endif
