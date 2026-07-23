#include "task/actions/tape_alignment_action.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static const float kMinControlDtS = 0.0005f;
static const uint8_t kStableSamplesRequired = 3U;

static TaskActionResult fail(TapeAlignmentAction *action,
                             TaskFailure failure) {
    (void)drivetrain_brake(action->drivetrain);
    action->result =
        (TaskActionResult){TASK_STEP_FAILED, failure};
    return action->result;
}

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

bool tape_alignment_action_start(TapeAlignmentAction *action,
                                 const TaskStepCommand *command,
                                 uint32_t now_ms) {
    if (action == NULL || command == NULL ||
        action->result.status == TASK_STEP_RUNNING ||
        (command->action != TASK_ACTION_ALIGN_TO_PIECES &&
         command->action != TASK_ACTION_ALIGN_TO_TAPE)) {
        return false;
    }
    if (action->drivetrain != NULL &&
        action->drivetrain->status.initialized &&
        !action->drivetrain->status.enabled &&
        drivetrain_enable(action->drivetrain) != ESP_OK) {
        return false;
    }

    action->selected_sensor =
        command->action == TASK_ACTION_ALIGN_TO_PIECES
            ? TAPE_FOLLOWER_FRONT
            : TAPE_FOLLOWER_BACK;
    tape_line_estimator_reset(&action->estimator_state);
    (void)tape_following_controller_reset(&action->controller_state);
    action->stable_samples = 0U;
    action->last_update_ms = now_ms;
    action->result =
        (TaskActionResult){TASK_STEP_RUNNING, TASK_FAILURE_NONE};
    return true;
}

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
    if (tape_sensor_driver_read_all(sensors) != ESP_OK) {
        return fail(action, TASK_FAILURE_STEP_FAILED);
    }

    const TapeSensor *sensor =
        action->selected_sensor == TAPE_FOLLOWER_FRONT
            ? action->front_sensor
            : action->back_sensor;
    const bool centered = !sensor->channel_0 && sensor->channel_1 &&
                          sensor->channel_2 && !sensor->channel_3;
    if (centered) {
        (void)drivetrain_set_body_velocity(action->drivetrain, 0.0f, 0.0f,
                                           0.0f);
        action->stable_samples++;
        if (action->stable_samples >= kStableSamplesRequired) {
            (void)drivetrain_stop(action->drivetrain);
            action->result =
                (TaskActionResult){TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
        }
        return action->result;
    }
    action->stable_samples = 0U;

    const TapeLineEstimatorConfig *estimator =
        action->tape_follower->config->estimators[action->selected_sensor];
    float line_error = 0.0f;
    if (!tape_line_estimator_compute_error(
            sensor, estimator, &action->estimator_state, &line_error)) {
        return fail(action, TASK_FAILURE_STEP_FAILED);
    }
    const float correction = tape_following_controller_update(
        &action->controller_state,
        &action->tape_follower->config->controller, line_error, dt_s);
    if (drivetrain_set_body_velocity(action->drivetrain, 0.0f, correction,
                                     0.0f) != ESP_OK) {
        return fail(action, TASK_FAILURE_STEP_FAILED);
    }
    return action->result;
}

void tape_alignment_action_cancel(TapeAlignmentAction *action) {
    if (action == NULL) return;
    if (action->drivetrain != NULL &&
        action->drivetrain->status.initialized) {
        (void)drivetrain_brake(action->drivetrain);
    }
    action->result =
        (TaskActionResult){TASK_STEP_CANCELLED, TASK_FAILURE_NONE};
}

bool tape_alignment_action_report_succeeded(TapeAlignmentAction *action) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING) {
        return false;
    }
    action->result =
        (TaskActionResult){TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    return true;
}

bool tape_alignment_action_report_failed(TapeAlignmentAction *action,
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
    return tape_alignment_action_start((TapeAlignmentAction *)context, command,
                                       now_ms);
}

static TaskActionResult handler_update(void *context, uint32_t now_ms) {
    return tape_alignment_action_update((TapeAlignmentAction *)context, now_ms);
}

static void handler_cancel(void *context, uint32_t now_ms) {
    (void)now_ms;
    tape_alignment_action_cancel((TapeAlignmentAction *)context);
}

static bool handler_report_succeeded(void *context) {
    return tape_alignment_action_report_succeeded(
        (TapeAlignmentAction *)context);
}

static bool handler_report_failed(void *context, TaskFailure failure) {
    return tape_alignment_action_report_failed((TapeAlignmentAction *)context,
                                               failure);
}

DrivetrainActionHandler tape_alignment_action_handler(
    TapeAlignmentAction *action) {
    return (DrivetrainActionHandler){
        .supported_actions =
            TASK_ACTION_BIT(TASK_ACTION_ALIGN_TO_PIECES) |
            TASK_ACTION_BIT(TASK_ACTION_ALIGN_TO_TAPE),
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
