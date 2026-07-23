#include "task/actions/follow_tape_action.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static const float kTwoPi = 6.283185307179586f;
static const float kMinControlDtS = 0.0005f;

static void capture_encoder_counts(const FollowTapeAction *action,
                                   int32_t counts_out[DRIVETRAIN_MOTOR_MAX]) {
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        counts_out[i] = drivetrain_get_encoder_accumulated_count(
            action->drivetrain, (DrivetrainMotorId)i);
    }
}

static esp_err_t integrate_odometry_step(FollowTapeAction *action) {
    int32_t new_counts[DRIVETRAIN_MOTOR_MAX];
    capture_encoder_counts(action, new_counts);

    float wheel_angle_delta_rad[DRIVETRAIN_MOTOR_MAX];
    for (int i = 0; i < DRIVETRAIN_MOTOR_MAX; ++i) {
        const int32_t delta_counts =
            new_counts[i] - action->last_encoder_counts[i];
        const uint32_t counts_per_rev =
            action->drivetrain->config->encoder_configs[i]
                ->counts_per_revolution;
        wheel_angle_delta_rad[i] =
            (float)delta_counts * kTwoPi / (float)counts_per_rev;
    }
    const XDriveWheelVelocity wheel_delta = {
        .fl = wheel_angle_delta_rad[DRIVETRAIN_MOTOR_FL],
        .fr = wheel_angle_delta_rad[DRIVETRAIN_MOTOR_FR],
        .bl = wheel_angle_delta_rad[DRIVETRAIN_MOTOR_BL],
        .br = wheel_angle_delta_rad[DRIVETRAIN_MOTOR_BR],
    };

    DrivetrainBodyVelocity body_delta = {0};
    const esp_err_t error = x_drive_kinematics_wheel_to_body_velocities(
        &action->drivetrain->config->x_drive_kinematics, &wheel_delta,
        &body_delta);
    if (error != ESP_OK) return error;

    memcpy(action->last_encoder_counts, new_counts, sizeof(new_counts));
    const DrivetrainOdometryDelta delta = {
        .forward_mm = body_delta.vx * 1000.0f,
        .lateral_mm = body_delta.vy * 1000.0f,
        .heading_delta_rad = body_delta.omega,
    };
    return drivetrain_odometry_update(&action->odometry, &delta, true);
}

static TaskActionResult fail(FollowTapeAction *action, TaskFailure failure) {
    (void)drivetrain_brake(action->drivetrain);
    action->result.status = TASK_STEP_FAILED;
    action->result.failure = failure;
    return action->result;
}

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

bool follow_tape_action_start(FollowTapeAction *action,
                              const TaskStepCommand *command,
                              uint32_t now_ms) {
    if (action == NULL || command == NULL ||
        command->action != TASK_ACTION_FOLLOW_TAPE ||
        action->result.status == TASK_STEP_RUNNING) {
        return false;
    }
    const TapeFollowingTaskParams *params = &command->tape_following;
    if (!isfinite(params->speed_mps) || params->speed_mps <= 0.0f ||
        !isfinite(params->distance_m) || params->distance_m <= 0.0f) {
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
    capture_encoder_counts(action, action->last_encoder_counts);
    action->last_update_ms = now_ms;
    action->target_distance_m = params->distance_m;
    action->signed_travel_speed_mps =
        params->direction == TAPE_DIRECTION_FORWARD ? params->speed_mps
                                                     : -params->speed_mps;
    action->result =
        (TaskActionResult){TASK_STEP_RUNNING, TASK_FAILURE_NONE};
    return true;
}

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

    const float traveled_m =
        hypotf(action->odometry.pose.x_mm, action->odometry.pose.y_mm) /
        1000.0f;
    if (traveled_m >= action->target_distance_m) {
        (void)drivetrain_stop(action->drivetrain);
        action->result =
            (TaskActionResult){TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    }
    return action->result;
}

void follow_tape_action_cancel(FollowTapeAction *action) {
    if (action == NULL) return;
    if (action->drivetrain != NULL &&
        action->drivetrain->status.initialized) {
        (void)drivetrain_brake(action->drivetrain);
    }
    action->result =
        (TaskActionResult){TASK_STEP_CANCELLED, TASK_FAILURE_NONE};
}

bool follow_tape_action_report_succeeded(FollowTapeAction *action) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING) {
        return false;
    }
    action->result =
        (TaskActionResult){TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    return true;
}

bool follow_tape_action_report_failed(FollowTapeAction *action,
                                      TaskFailure failure) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING ||
        failure == TASK_FAILURE_NONE) {
        return false;
    }
    (void)fail(action, failure);
    return true;
}

static bool handler_start(void *context, const TaskStepCommand *command,
                          uint32_t now_ms) {
    return follow_tape_action_start((FollowTapeAction *)context, command,
                                    now_ms);
}

static TaskActionResult handler_update(void *context, uint32_t now_ms) {
    return follow_tape_action_update((FollowTapeAction *)context, now_ms);
}

static void handler_cancel(void *context, uint32_t now_ms) {
    (void)now_ms;
    follow_tape_action_cancel((FollowTapeAction *)context);
}

static bool handler_report_succeeded(void *context) {
    return follow_tape_action_report_succeeded((FollowTapeAction *)context);
}

static bool handler_report_failed(void *context, TaskFailure failure) {
    return follow_tape_action_report_failed((FollowTapeAction *)context,
                                            failure);
}

DrivetrainActionHandler follow_tape_action_handler(FollowTapeAction *action) {
    return (DrivetrainActionHandler){
        .supported_actions = TASK_ACTION_BIT(TASK_ACTION_FOLLOW_TAPE),
        .executor =
            {
                .context = action,
                .start = handler_start,
                .update = handler_update,
                .cancel = handler_cancel,
            },
        .report_succeeded = handler_report_succeeded,
        .report_failed = handler_report_failed,
    };
}
