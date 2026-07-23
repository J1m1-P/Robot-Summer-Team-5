/**
 * @file tape_alignment_action.c
 * @brief Implements the drivetrain phases used around tower acquisition.
 *
 * A command action selects one of five nonblocking behaviors: initial
 * alignment, approach following, task-tape travel, backing away, or main-route
 * reacquisition. Shared sensor, controller, and odometry primitives keep those
 * related behaviors in one editable module.
 */
#include "task/actions/tape_alignment_action.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

// Control-loop floor and number of centered readings required for alignment.
static const float kMinControlDtS = 0.0005f;
static const uint8_t kStableSamplesRequired = 3U;

// Internal phases used by seek-and-center alignment behaviors.
enum {
    PHASE_PRIMARY = 0,
    PHASE_SEEK_TAPE,
    PHASE_ALIGN_TAPE,
};

// Brakes motion and stores a terminal failed result.
static TaskActionResult fail(TapeAlignmentAction *action,
                             TaskFailure failure) {
    (void)drivetrain_brake(action->drivetrain);
    action->result =
        (TaskActionResult){TASK_STEP_FAILED, failure};
    return action->result;
}

// Returns true for action IDs intentionally owned by this implementation.
static bool supported_action(TaskAction action) {
    return action == TASK_ACTION_ALIGN_TO_PIECES ||
           action == TASK_ACTION_FOLLOW_PIECES_TAPE ||
           action == TASK_ACTION_FOLLOW_TASK_TAPE ||
           action == TASK_ACTION_BACK_OFF_PIECES ||
           action == TASK_ACTION_ALIGN_TO_TAPE;
}

// Treats any active channel as tape presence for seek/edge detection.
static bool tape_present(const TapeSensor *sensor) {
    return sensor != NULL &&
           (sensor->channel_0 || sensor->channel_1 ||
            sensor->channel_2 || sensor->channel_3);
}

// Requires the two center channels, and no outer channels, for alignment.
static bool tape_centered(const TapeSensor *sensor) {
    return sensor != NULL && !sensor->channel_0 && sensor->channel_1 &&
           sensor->channel_2 && !sensor->channel_3;
}

// Samples accumulated quadrature counts for all four drivetrain wheels.
static DrivetrainWheelCounts wheel_counts(
    const TapeAlignmentAction *action) {
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
static bool configure_odometry(TapeAlignmentAction *action) {
    if (action->drivetrain == NULL || action->drivetrain->config == NULL) {
        return false;
    }
    const DrivetrainConfig *config = action->drivetrain->config;
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        if (config->encoder_configs[i] == NULL) return false;
    }
    action->odometry_source_config = (DrivetrainOdometrySourceConfig){
        .x_drive_kinematics = config->x_drive_kinematics,
        .counts_per_revolution_fl =
            config->encoder_configs[DRIVETRAIN_MOTOR_FL]
                ->counts_per_revolution,
        .counts_per_revolution_fr =
            config->encoder_configs[DRIVETRAIN_MOTOR_FR]
                ->counts_per_revolution,
        .counts_per_revolution_bl =
            config->encoder_configs[DRIVETRAIN_MOTOR_BL]
                ->counts_per_revolution,
        .counts_per_revolution_br =
            config->encoder_configs[DRIVETRAIN_MOTOR_BR]
                ->counts_per_revolution,
    };
    return drivetrain_odometry_source_config_is_valid(
        &action->odometry_source_config);
}

// Integrates one encoder sample and accumulates path length in metres.
static esp_err_t update_odometry(TapeAlignmentAction *action) {
    const DrivetrainPose previous = action->odometry.pose;
    const DrivetrainWheelCounts counts = wheel_counts(action);
    const esp_err_t error = drivetrain_odometry_source_update(
        &action->odometry_source, &action->odometry_source_config, &counts,
        &action->odometry);
    if (error != ESP_OK || action->odometry.fault_latched) {
        return error != ESP_OK ? error : ESP_FAIL;
    }
    action->traveled_distance_m +=
        hypotf(action->odometry.pose.x_mm - previous.x_mm,
               action->odometry.pose.y_mm - previous.y_mm) /
        1000.0f;
    return ESP_OK;
}

// Stops commanded motion and stores a successful terminal result.
static TaskActionResult succeed(TapeAlignmentAction *action) {
    (void)drivetrain_stop(action->drivetrain);
    action->result =
        (TaskActionResult){TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    return action->result;
}

// Centers the left module using the line estimator and bounded angular control.
static TaskActionResult align_with_left_sensor(TapeAlignmentAction *action,
                                               float dt_s) {
    if (tape_centered(action->left_sensor)) {
        (void)drivetrain_set_body_velocity(action->drivetrain, 0.0f, 0.0f,
                                           0.0f);
        if (++action->stable_samples >= kStableSamplesRequired) {
            return succeed(action);
        }
        return action->result;
    }
    action->stable_samples = 0U;

    float error = 0.0f;
    const TapeLineEstimatorConfig *estimator =
        action->tape_follower->config->estimators[TAPE_FOLLOWER_FRONT];
    if (!tape_line_estimator_compute_error(
            action->left_sensor, estimator, &action->estimator_state,
            &error)) {
        action->phase = PHASE_SEEK_TAPE;
        return action->result;
    }
    float omega = tape_following_controller_update(
        &action->controller_state,
        &action->tape_follower->config->controller, error, dt_s);
    const float limit = fabsf(action->parameters.speed);
    omega = fmaxf(-limit, fminf(limit, omega));
    if (drivetrain_set_body_velocity(action->drivetrain, 0.0f, 0.0f,
                                     omega) != ESP_OK) {
        return fail(action, TASK_FAILURE_STEP_FAILED);
    }
    return action->result;
}

// Binds borrowed hardware/controller objects and resets per-run state.
void tape_alignment_action_init(TapeAlignmentAction *action,
                                Drivetrain *drivetrain,
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

// Validates generic parameters and initializes state for the selected behavior.
bool tape_alignment_action_start(TapeAlignmentAction *action,
                                 const TaskStepCommand *command,
                                 uint32_t now_ms) {
    if (action == NULL || command == NULL ||
        action->result.status == TASK_STEP_RUNNING ||
        !supported_action(command->action) ||
        !isfinite(command->parameters.amount) ||
        !isfinite(command->parameters.speed) ||
        command->parameters.speed < 0.0f ||
        !configure_odometry(action)) {
        return false;
    }
    if (action->drivetrain != NULL &&
        action->drivetrain->status.initialized &&
        !action->drivetrain->status.enabled &&
        drivetrain_enable(action->drivetrain) != ESP_OK) {
        return false;
    }

    action->active_action = command->action;
    action->parameters = command->parameters;
    action->phase = PHASE_PRIMARY;
    if (command->action == TASK_ACTION_ALIGN_TO_PIECES) {
        action->task_tape_detected = false;
    } else if (command->action == TASK_ACTION_BACK_OFF_PIECES) {
        action->route_tape_detected = false;
        action->route_tape_cleared = false;
    }
    tape_line_estimator_reset(&action->estimator_state);
    (void)tape_following_controller_reset(&action->controller_state);
    (void)tape_follower_reset(action->tape_follower);
    drivetrain_odometry_reset(&action->odometry);
    drivetrain_odometry_source_reset(&action->odometry_source);
    action->traveled_distance_m = 0.0f;
    if (update_odometry(action) != ESP_OK) return false;
    action->stable_samples = 0U;
    action->last_update_ms = now_ms;
    action->result =
        (TaskActionResult){TASK_STEP_RUNNING, TASK_FAILURE_NONE};
    return true;
}

// Reads sensors/odometry and advances the active behavior by one loop cycle.
TaskActionResult tape_alignment_action_update(TapeAlignmentAction *action,
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
    if (tape_sensor_driver_read_all(sensors) != ESP_OK ||
        update_odometry(action) != ESP_OK) {
        return fail(action, TASK_FAILURE_STEP_FAILED);
    }

    // Tower picking 1: turn clockwise by the requested angle, continue
    // clockwise until the left module finds tape, then center on that tape.
    if (action->active_action == TASK_ACTION_ALIGN_TO_PIECES) {
        if (action->phase == PHASE_PRIMARY) {
            if (action->odometry.pose.heading_rad <=
                -fabsf(action->parameters.amount)) {
                action->phase = PHASE_SEEK_TAPE;
            } else if (drivetrain_set_body_velocity(
                           action->drivetrain, 0.0f, 0.0f,
                           -action->parameters.speed) != ESP_OK) {
                return fail(action, TASK_FAILURE_STEP_FAILED);
            }
            return action->result;
        }
        if (action->phase == PHASE_SEEK_TAPE) {
            if (tape_present(action->left_sensor)) {
                action->phase = PHASE_ALIGN_TAPE;
            } else if (drivetrain_set_body_velocity(
                           action->drivetrain, 0.0f, 0.0f,
                           -action->parameters.speed) != ESP_OK) {
                return fail(action, TASK_FAILURE_STEP_FAILED);
            }
            return action->result;
        }
        return align_with_left_sensor(action, dt_s);
    }

    // Temporary adapters for the future interrupt-based tape detector. The
    // notification functions preserve the workflow boundary when the GPIO
    // interrupt implementation replaces these sampled edges.
    if (action->active_action == TASK_ACTION_FOLLOW_PIECES_TAPE &&
        tape_present(action->back_sensor)) {
        tape_alignment_action_notify_task_tape(action);
    }
    if (action->active_action == TASK_ACTION_BACK_OFF_PIECES) {
        if (!tape_present(action->left_sensor)) {
            action->route_tape_cleared = true;
        } else if (action->route_tape_cleared) {
            tape_alignment_action_notify_route_tape(action);
        }
    }

    // Tower picking 2: use the left module as the leading guidance module
    // until the latched task-tape interrupt ends this nondeterministic leg.
    if (action->active_action == TASK_ACTION_FOLLOW_PIECES_TAPE) {
        if (action->task_tape_detected) return succeed(action);
        const TapeFollowerInput input = {
            {action->left_sensor, action->left_sensor},
            action->parameters.speed,
        };
        TapeFollowerOutput output = {0};
        if (tape_follower_update(action->tape_follower, &input, dt_s,
                                 &output) != ESP_OK ||
            output.status == TAPE_FOLLOWER_LOST || !output.motion_valid ||
            drivetrain_set_body_velocity(
                action->drivetrain, output.requested_velocity.vx,
                output.requested_velocity.vy,
                output.requested_velocity.omega) != ESP_OK) {
            return fail(action, TASK_FAILURE_STEP_FAILED);
        }
        return action->result;
    }

    // Tower picking 3: follow with the back module for the requested distance.
    if (action->active_action == TASK_ACTION_FOLLOW_TASK_TAPE) {
        if (action->traveled_distance_m >= action->parameters.amount) {
            return succeed(action);
        }
        const TapeFollowerInput input = {
            {action->front_sensor, action->back_sensor},
            -action->parameters.speed,
        };
        TapeFollowerOutput output = {0};
        if (tape_follower_update(action->tape_follower, &input, dt_s,
                                 &output) != ESP_OK ||
            output.status == TAPE_FOLLOWER_LOST || !output.motion_valid ||
            drivetrain_set_body_velocity(
                action->drivetrain, output.requested_velocity.vx,
                output.requested_velocity.vy,
                output.requested_velocity.omega) != ESP_OK) {
            return fail(action, TASK_FAILURE_STEP_FAILED);
        }
        return action->result;
    }

    // Tower picking 11: back off by at least the requested distance, then
    // continue until the latched route-tape interrupt permits realignment.
    if (action->active_action == TASK_ACTION_BACK_OFF_PIECES) {
        if (action->traveled_distance_m >= action->parameters.amount &&
            action->route_tape_detected) {
            return succeed(action);
        }
        if (drivetrain_set_body_velocity(action->drivetrain,
                                         -action->parameters.speed, 0.0f,
                                         0.0f) != ESP_OK) {
            return fail(action, TASK_FAILURE_STEP_FAILED);
        }
        return action->result;
    }

    // Tower picking 12: find the main tape with the left module and center it.
    if (action->phase != PHASE_ALIGN_TAPE &&
        !tape_present(action->left_sensor)) {
        if (drivetrain_set_body_velocity(action->drivetrain, 0.0f, 0.0f,
                                         -action->parameters.speed) != ESP_OK) {
            return fail(action, TASK_FAILURE_STEP_FAILED);
        }
        return action->result;
    }
    action->phase = PHASE_ALIGN_TAPE;
    return align_with_left_sensor(action, dt_s);
}

// Immediately brakes and publishes a cancelled terminal result.
void tape_alignment_action_cancel(TapeAlignmentAction *action) {
    if (action == NULL) return;
    if (action->drivetrain != NULL &&
        action->drivetrain->status.initialized) {
        (void)drivetrain_brake(action->drivetrain);
    }
    action->result =
        (TaskActionResult){TASK_STEP_CANCELLED, TASK_FAILURE_NONE};
}

// Allows the supervised test harness to force successful completion.
bool tape_alignment_action_report_succeeded(TapeAlignmentAction *action) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING) {
        return false;
    }
    action->result =
        (TaskActionResult){TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    return true;
}

// Allows the supervised test harness to inject a specific action failure.
bool tape_alignment_action_report_failed(TapeAlignmentAction *action,
                                         TaskFailure failure) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING ||
        failure == TASK_FAILURE_NONE) {
        return false;
    }
    (void)fail(action, failure);
    return true;
}
