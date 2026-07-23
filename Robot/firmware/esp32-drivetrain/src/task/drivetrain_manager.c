#include "task/drivetrain_manager.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include <robot_common/app_log.h>

// Avoid the portability issues of M_PI; matches the constant used by the
// drivetrain hardware test harness for the same purpose.
static const float kTwoPi = 6.283185307179586f;
// Floors the control-loop dt so a scheduling stall or repeated now_ms cannot
// produce a zero or negative dt into the tape follower.
static const float kMinControlDtS = 0.0005f;

// Keeps physical drivetrain actions from being accepted by the arm-side executor.
static bool action_is_drivetrain_owned(TaskAction action) {
    return action == TASK_ACTION_FOLLOW_TAPE ||
           action == TASK_ACTION_ALIGN_TO_PIECES ||
           action == TASK_ACTION_ALIGN_TO_TAPE ||
           action == TASK_ACTION_ALIGN_TO_BASE;
}

static void capture_encoder_counts(DrivetrainManager *manager,
                                   int32_t counts_out[DRIVETRAIN_MOTOR_MAX]) {
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        counts_out[i] = drivetrain_get_encoder_accumulated_count(
            manager->drivetrain, (DrivetrainMotorId)i);
    }
}

// Converts this cycle's encoder deltas into a body-frame displacement and
// folds it into the manager's odometry. Reuses the drivetrain's own
// velocity Jacobian, but applied to angle deltas rather than angular rates,
// which is exact for a linear, time-invariant kinematics map.
static esp_err_t integrate_odometry_step(DrivetrainManager *manager) {
    int32_t new_counts[DRIVETRAIN_MOTOR_MAX];
    capture_encoder_counts(manager, new_counts);

    float wheel_angle_delta_rad[DRIVETRAIN_MOTOR_MAX];
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        const int32_t delta_counts = new_counts[i] - manager->last_encoder_counts[i];
        const uint32_t counts_per_rev =
            manager->drivetrain->config->encoder_configs[i]->counts_per_revolution;
        wheel_angle_delta_rad[i] = (float)delta_counts * kTwoPi /
                                   (float)counts_per_rev;
    }
    const XDriveWheelVelocity wheel_delta = {
        .fl = wheel_angle_delta_rad[DRIVETRAIN_MOTOR_FL],
        .fr = wheel_angle_delta_rad[DRIVETRAIN_MOTOR_FR],
        .bl = wheel_angle_delta_rad[DRIVETRAIN_MOTOR_BL],
        .br = wheel_angle_delta_rad[DRIVETRAIN_MOTOR_BR],
    };

    DrivetrainBodyVelocity body_delta = {0};
    const esp_err_t error = x_drive_kinematics_wheel_to_body_velocities(
        &manager->drivetrain->config->x_drive_kinematics, &wheel_delta,
        &body_delta);
    if (error != ESP_OK) return error;

    memcpy(manager->last_encoder_counts, new_counts, sizeof(new_counts));

    const DrivetrainOdometryDelta delta = {
        .forward_mm = body_delta.vx * 1000.0f,
        .lateral_mm = body_delta.vy * 1000.0f,
        .heading_delta_rad = body_delta.omega,
    };
    return drivetrain_odometry_update(&manager->odometry, &delta, true);
}

static TaskActionResult fail_follow_tape(DrivetrainManager *manager,
                                         TaskFailure failure) {
    (void)drivetrain_stop(manager->drivetrain);
    manager->result.status = TASK_STEP_FAILED;
    manager->result.failure = failure;
    return manager->result;
}

// Samples tape sensors, steers/paces via TapeFollower, and reports success
// once accumulated odometry reaches the requested distance.
static TaskActionResult update_follow_tape(DrivetrainManager *manager,
                                           uint32_t now_ms) {
    const float dt_s = fmaxf(
        kMinControlDtS, (float)(now_ms - manager->last_update_ms) / 1000.0f);
    manager->last_update_ms = now_ms;

    TapeSensor *sensors[TAPE_SENSOR_MODULE_COUNT] = {
        &manager->tape_sensor_front, &manager->tape_sensor_back,
        &manager->tape_sensor_left,
    };
    if (tape_sensor_driver_read_all(sensors) != ESP_OK) {
        return fail_follow_tape(manager, TASK_FAILURE_STEP_FAILED);
    }

    const TapeFollowerInput input = {
        {&manager->tape_sensor_front, &manager->tape_sensor_back},
        manager->signed_travel_speed_mps,
    };
    TapeFollowerOutput output = {0};
    if (tape_follower_update(&manager->tape_follower, &input, dt_s, &output) !=
            ESP_OK ||
        output.status == TAPE_FOLLOWER_LOST || !output.motion_valid) {
        return fail_follow_tape(manager, TASK_FAILURE_STEP_FAILED);
    }

    if (drivetrain_set_body_velocity(manager->drivetrain,
                                     output.requested_velocity.vx,
                                     output.requested_velocity.vy,
                                     output.requested_velocity.omega) != ESP_OK ||
        integrate_odometry_step(manager) != ESP_OK) {
        return fail_follow_tape(manager, TASK_FAILURE_STEP_FAILED);
    }

    const float traveled_m = hypotf(manager->odometry.pose.x_mm,
                                    manager->odometry.pose.y_mm) / 1000.0f;
    if (traveled_m >= manager->target_distance_m) {
        (void)drivetrain_stop(manager->drivetrain);
        manager->result.status = TASK_STEP_SUCCEEDED;
        manager->result.failure = TASK_FAILURE_NONE;
    }
    return manager->result;
}

// Accepts one drivetrain-owned action when no action is already running.
static bool manager_start(void *context, const TaskStepCommand *command,
                          uint32_t now_ms) {
    DrivetrainManager *manager = (DrivetrainManager *)context;
    if (manager == NULL || command == NULL ||
        manager->result.status == TASK_STEP_RUNNING ||
        !action_is_drivetrain_owned(command->action)) {
        return false;
    }

    const bool is_follow_tape = command->action == TASK_ACTION_FOLLOW_TAPE;
    const TapeFollowingTaskParams *params = &command->tape_following;
    if (is_follow_tape &&
        (!manager->tape_hardware_ready || !isfinite(params->speed_mps) ||
         params->speed_mps <= 0.0f || !isfinite(params->distance_m) ||
         params->distance_m <= 0.0f)) {
        return false;
    }

    // The drivetrain boots braked and disabled (see main.cpp); this is the
    // sole point that wakes it for physical motion, and only after every
    // rejection check above has passed. Cancel/report_failed already
    // re-brake it, so it only stays live while a step is running.
    if (manager->drivetrain != NULL && manager->drivetrain->status.initialized &&
        !manager->drivetrain->status.enabled &&
        drivetrain_enable(manager->drivetrain) != ESP_OK) {
        return false;
    }

    manager->active_action = command->action;
    if (is_follow_tape) {
        (void)tape_follower_reset(&manager->tape_follower);
        drivetrain_odometry_reset(&manager->odometry);
        capture_encoder_counts(manager, manager->last_encoder_counts);
        manager->last_update_ms = now_ms;
        manager->target_distance_m = params->distance_m;
        manager->signed_travel_speed_mps =
            params->direction == TAPE_DIRECTION_FORWARD ? params->speed_mps
                                                         : -params->speed_mps;
    }

    manager->result.status = TASK_STEP_RUNNING;
    manager->result.failure = TASK_FAILURE_NONE;
    return true;
}

// Returns the latest result; drives the real state machine for follow-tape
// and passes through the placeholder logical status for other actions.
static TaskActionResult manager_update(void *context, uint32_t now_ms) {
    DrivetrainManager *manager = (DrivetrainManager *)context;
    if (manager == NULL || manager->result.status == TASK_STEP_NOT_STARTED) {
        const TaskActionResult invalid = {
            TASK_STEP_FAILED, TASK_FAILURE_PROTOCOL};
        return invalid;
    }
    if (manager->result.status == TASK_STEP_RUNNING &&
        manager->active_action == TASK_ACTION_FOLLOW_TAPE) {
        return update_follow_tape(manager, now_ms);
    }
    return manager->result;
}

// Brakes initialized hardware before publishing a cancelled result.
static void manager_cancel(void *context, uint32_t now_ms) {
    (void)now_ms;
    DrivetrainManager *manager = (DrivetrainManager *)context;
    if (manager == NULL) return;
    if (manager->drivetrain != NULL &&
        manager->drivetrain->status.initialized) {
        (void)drivetrain_brake(manager->drivetrain);
    }
    manager->result.status = TASK_STEP_CANCELLED;
    manager->result.failure = TASK_FAILURE_NONE;
}

// Initializes tape-following hardware once and binds the drivetrain used for
// fail-safe braking. Hardware failures are logged and leave
// tape_hardware_ready false rather than aborting the whole manager.
esp_err_t drivetrain_manager_init(DrivetrainManager *manager,
                                  Drivetrain *drivetrain,
                                  const TapeSensorMuxConfig *tape_mux_config,
                                  const TapeSensorDriverConfig *front_sensor_config,
                                  const TapeSensorDriverConfig *back_sensor_config,
                                  const TapeSensorDriverConfig *left_sensor_config,
                                  const TapeFollowerConfig *tape_follower_config) {
    if (manager == NULL) return ESP_ERR_INVALID_ARG;
    memset(manager, 0, sizeof(*manager));
    manager->drivetrain = drivetrain;
    manager->result.status = TASK_STEP_NOT_STARTED;

    esp_err_t error = tape_sensor_mux_init(&manager->tape_mux, tape_mux_config);
    if (error == ESP_OK) {
        error = tape_sensor_driver_init(&manager->tape_sensor_front,
                                        front_sensor_config, &manager->tape_mux);
    }
    if (error == ESP_OK) {
        error = tape_sensor_driver_init(&manager->tape_sensor_back,
                                        back_sensor_config, &manager->tape_mux);
    }
    if (error == ESP_OK) {
        error = tape_sensor_driver_init(&manager->tape_sensor_left,
                                        left_sensor_config, &manager->tape_mux);
    }
    if (error == ESP_OK) {
        error = tape_follower_init(&manager->tape_follower, tape_follower_config);
    }
    if (error != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN,
                 "Tape-following hardware init failed; follow-tape action "
                 "disabled: %s",
                 esp_err_to_name(error));
        manager->tape_hardware_ready = false;
        return error;
    }

    manager->tape_hardware_ready = true;
    return ESP_OK;
}

// Adapts the concrete manager to the coordinator's generic executor callbacks.
TaskActionExecutor drivetrain_manager_executor(DrivetrainManager *manager) {
    const TaskActionExecutor executor = {
        .context = manager,
        .start = manager_start,
        .update = manager_update,
        .cancel = manager_cancel,
    };
    return executor;
}

// Lets nonblocking drivetrain behavior report successful physical completion.
bool drivetrain_manager_report_succeeded(DrivetrainManager *manager) {
    if (manager == NULL || manager->result.status != TASK_STEP_RUNNING) {
        return false;
    }
    manager->result.status = TASK_STEP_SUCCEEDED;
    return true;
}

// Records a physical failure and brakes the drivetrain immediately.
bool drivetrain_manager_report_failed(DrivetrainManager *manager,
                                      TaskFailure failure) {
    if (manager == NULL || manager->result.status != TASK_STEP_RUNNING ||
        failure == TASK_FAILURE_NONE) {
        return false;
    }
    manager->result.status = TASK_STEP_FAILED;
    manager->result.failure = failure;
    if (manager->drivetrain != NULL &&
        manager->drivetrain->status.initialized) {
        (void)drivetrain_brake(manager->drivetrain);
    }
    return true;
}
