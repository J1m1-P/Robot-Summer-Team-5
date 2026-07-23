/**
 * @file follow_tape_action.c
 * @brief Implements closed-loop tape following for a requested path length.
 *
 * The action combines tape-sensor guidance with drivetrain body-velocity
 * control and wheel-encoder odometry. All work is advanced from update(), so
 * the coordinator remains responsive to cancellation and timeouts.
 */
#include "task/actions/follow_tape_action.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// Small lower bound prevents a zero controller timestep on fast loop cycles.
static const float kMinControlDtS = 0.0005f;

// Samples accumulated quadrature counts for all four drivetrain wheels.
static DrivetrainWheelCounts capture_encoder_counts(
    const FollowTapeAction *action) {
    return (DrivetrainWheelCounts){
        .fl = drivetrain_get_encoder_accumulated_count(
            action->drivetrain, DRIVETRAIN_MOTOR_FL),
        .fr = drivetrain_get_encoder_accumulated_count(
            action->drivetrain, DRIVETRAIN_MOTOR_FR),
        .bl = drivetrain_get_encoder_accumulated_count(
            action->drivetrain, DRIVETRAIN_MOTOR_BL),
        .br = drivetrain_get_encoder_accumulated_count(
            action->drivetrain, DRIVETRAIN_MOTOR_BR),
    };
}

// Builds odometry conversion parameters from the live drivetrain configuration.
static bool configure_odometry_source(FollowTapeAction *action) {
    if (action == NULL || action->drivetrain == NULL ||
        action->drivetrain->config == NULL) {
        return false;
    }

    const DrivetrainConfig *drivetrain_config = action->drivetrain->config;
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        if (drivetrain_config->encoder_configs[i] == NULL) return false;
    }

    action->odometry_source_config = (DrivetrainOdometrySourceConfig){
        .x_drive_kinematics = drivetrain_config->x_drive_kinematics,
        .counts_per_revolution_fl =
            drivetrain_config->encoder_configs[DRIVETRAIN_MOTOR_FL]
                ->counts_per_revolution,
        .counts_per_revolution_fr =
            drivetrain_config->encoder_configs[DRIVETRAIN_MOTOR_FR]
                ->counts_per_revolution,
        .counts_per_revolution_bl =
            drivetrain_config->encoder_configs[DRIVETRAIN_MOTOR_BL]
                ->counts_per_revolution,
        .counts_per_revolution_br =
            drivetrain_config->encoder_configs[DRIVETRAIN_MOTOR_BR]
                ->counts_per_revolution,
    };
    return drivetrain_odometry_source_config_is_valid(
        &action->odometry_source_config);
}

// Integrates one encoder sample and accumulates curved path length in metres.
static esp_err_t integrate_odometry_step(FollowTapeAction *action) {
    const DrivetrainPose previous_pose = action->odometry.pose;
    const DrivetrainWheelCounts counts = capture_encoder_counts(action);
    const esp_err_t error = drivetrain_odometry_source_update(
        &action->odometry_source, &action->odometry_source_config, &counts,
        &action->odometry);
    if (error != ESP_OK || action->odometry.fault_latched) {
        return error != ESP_OK ? error : ESP_FAIL;
    }

    // Accumulate path length rather than start-to-current displacement so
    // bends in the tape count toward the requested travel distance.
    const float delta_x_mm = action->odometry.pose.x_mm - previous_pose.x_mm;
    const float delta_y_mm = action->odometry.pose.y_mm - previous_pose.y_mm;
    action->traveled_distance_m +=
        hypotf(delta_x_mm, delta_y_mm) / 1000.0f;
    return ESP_OK;
}

// Brakes motion and stores a terminal failed result.
static TaskActionResult fail(FollowTapeAction *action, TaskFailure failure) {
    (void)drivetrain_brake(action->drivetrain);
    action->result.status = TASK_STEP_FAILED;
    action->result.failure = failure;
    return action->result;
}

// Binds borrowed hardware/controller objects and resets per-run state.
void follow_tape_action_init(FollowTapeAction *action, Drivetrain *drivetrain,
                             TapeSensor *front_sensor,
                             TapeSensor *back_sensor,
                             TapeSensor *left_sensor,
                             TapeFollower *tape_follower) {
    if (action == NULL) return;
    memset(action, 0, sizeof(*action));
    action->drivetrain = drivetrain;
    action->front_sensor = front_sensor;
    action->back_sensor = back_sensor;
    action->left_sensor = left_sensor;
    action->tape_follower = tape_follower;
    action->result.status = TASK_STEP_NOT_STARTED;
}

// Validates speed/distance, enables motion, and captures odometry baseline.
bool follow_tape_action_start(FollowTapeAction *action,
                              const TaskStepCommand *command,
                              uint32_t now_ms) {
    if (action == NULL || command == NULL ||
        command->action != TASK_ACTION_FOLLOW_TAPE ||
        action->result.status == TASK_STEP_RUNNING) {
        return false;
    }
    const TaskStepParameters *params = &command->parameters;
    if (!isfinite(params->speed) || params->speed <= 0.0f ||
        !isfinite(params->amount) || params->amount == 0.0f ||
        !configure_odometry_source(action)) {
        return false;
    }
    if (action->drivetrain != NULL &&
        action->drivetrain->status.initialized &&
        !action->drivetrain->status.enabled &&
        drivetrain_enable(action->drivetrain) != ESP_OK) {
        return false;
    }

    (void)tape_follower_reset(action->tape_follower);
    drivetrain_odometry_reset(&action->odometry);
    drivetrain_odometry_source_reset(&action->odometry_source);
    action->traveled_distance_m = 0.0f;
    if (integrate_odometry_step(action) != ESP_OK) return false;
    action->last_update_ms = now_ms;
    action->target_distance_m = fabsf(params->amount);
    action->signed_travel_speed_mps =
        params->amount > 0.0f ? params->speed : -params->speed;
    action->result =
        (TaskActionResult){TASK_STEP_RUNNING, TASK_FAILURE_NONE};
    return true;
}

// Runs one sensing/control/odometry cycle and succeeds at target distance.
TaskActionResult follow_tape_action_update(FollowTapeAction *action,
                                           uint32_t now_ms) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING) {
        return action == NULL
                   ? (TaskActionResult){TASK_STEP_FAILED,
                                        TASK_FAILURE_PROTOCOL}
                   : action->result;
    }

    const float dt_s =
        fmaxf(kMinControlDtS,
              (float)(now_ms - action->last_update_ms) / 1000.0f);
    action->last_update_ms = now_ms;

    TapeSensor *sensors[TAPE_SENSOR_MODULE_COUNT] = {
        action->front_sensor, action->back_sensor, action->left_sensor};
    if (tape_sensor_driver_read_all(sensors) != ESP_OK) {
        return fail(action, TASK_FAILURE_STEP_FAILED);
    }

    const TapeFollowerInput input = {
        {action->front_sensor, action->back_sensor},
        action->signed_travel_speed_mps,
    };
    TapeFollowerOutput output = {0};
    if (tape_follower_update(action->tape_follower, &input, dt_s, &output) !=
            ESP_OK ||
        output.status == TAPE_FOLLOWER_LOST || !output.motion_valid) {
        return fail(action, TASK_FAILURE_STEP_FAILED);
    }
    if (drivetrain_set_body_velocity(action->drivetrain,
                                     output.requested_velocity.vx,
                                     output.requested_velocity.vy,
                                     output.requested_velocity.omega) != ESP_OK ||
        integrate_odometry_step(action) != ESP_OK) {
        return fail(action, TASK_FAILURE_STEP_FAILED);
    }

    if (action->traveled_distance_m >= action->target_distance_m) {
        (void)drivetrain_stop(action->drivetrain);
        action->result =
            (TaskActionResult){TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    }
    return action->result;
}

// Immediately brakes and publishes a cancelled terminal result.
void follow_tape_action_cancel(FollowTapeAction *action) {
    if (action == NULL) return;
    if (action->drivetrain != NULL &&
        action->drivetrain->status.initialized) {
        (void)drivetrain_brake(action->drivetrain);
    }
    action->result =
        (TaskActionResult){TASK_STEP_CANCELLED, TASK_FAILURE_NONE};
}

// Allows the supervised test harness to force successful completion.
bool follow_tape_action_report_succeeded(FollowTapeAction *action) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING) {
        return false;
    }
    action->result =
        (TaskActionResult){TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    return true;
}

// Allows the supervised test harness to inject a specific action failure.
bool follow_tape_action_report_failed(FollowTapeAction *action,
                                      TaskFailure failure) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING ||
        failure == TASK_FAILURE_NONE) {
        return false;
    }
    (void)fail(action, failure);
    return true;
}
