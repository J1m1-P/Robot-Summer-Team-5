/**
 * @file drivetrain_manager.c
 * @brief Initializes shared tape hardware and directly routes drive actions.
 *
 * This manager is the coordinator-facing boundary for all drivetrain-owned
 * task actions. It selects an independent action module from the command,
 * forwards update/cancel/result hooks, and rejects work when hardware is busy
 * or unavailable.
 */
#include "task/drivetrain_manager.h"

#include <stddef.h>
#include <string.h>

#include <robot_common/app_log.h>

// Groups the five picking-route action IDs handled by TapeAlignmentAction.
static bool is_alignment_action(TaskAction action) {
    return action == TASK_ACTION_ALIGN_TO_PIECES ||
           action == TASK_ACTION_FOLLOW_PIECES_TAPE ||
           action == TASK_ACTION_FOLLOW_TASK_TAPE ||
           action == TASK_ACTION_BACK_OFF_PIECES ||
           action == TASK_ACTION_ALIGN_TO_TAPE;
}

// Initializes shared hardware and idle action instances; failure disables routing.
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
    manager->active_action = TASK_ACTION_COUNT;

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
        error = tape_follower_init(&manager->tape_follower,
                                   tape_follower_config);
    }

    follow_tape_action_init(
        &manager->follow_tape, drivetrain, &manager->tape_sensor_front,
        &manager->tape_sensor_back, &manager->tape_sensor_left,
        &manager->tape_follower);
    tape_alignment_action_init(
        &manager->tape_alignment, drivetrain, &manager->tape_sensor_front,
        &manager->tape_sensor_back, &manager->tape_sensor_left,
        &manager->tape_follower);

    if (error != ESP_OK) {
        APP_LOGE(LOG_TAG_DRIVETRAIN,
                 "Tape action hardware initialization failed; drivetrain "
                 "actions disabled: %s",
                 esp_err_to_name(error));
        manager->tape_hardware_ready = false;
        return error;
    }

    manager->tape_hardware_ready = true;
    return ESP_OK;
}

// Selects and starts the one local action matching command->action.
static bool executor_start(void *context, const TaskStepCommand *command,
                           uint32_t now_ms) {
    DrivetrainManager *manager = (DrivetrainManager *)context;
    if (manager == NULL || command == NULL || !manager->tape_hardware_ready ||
        manager->active_action != TASK_ACTION_COUNT) {
        return false;
    }

    bool started = false;
    if (command->action == TASK_ACTION_FOLLOW_TAPE) {
        started =
            follow_tape_action_start(&manager->follow_tape, command, now_ms);
    } else if (is_alignment_action(command->action)) {
        started = tape_alignment_action_start(&manager->tape_alignment,
                                              command, now_ms);
    }
    if (started) manager->active_action = command->action;
    return started;
}

// Polls the selected action and releases it after a terminal result.
static TaskActionResult executor_update(void *context, uint32_t now_ms) {
    DrivetrainManager *manager = (DrivetrainManager *)context;
    if (manager == NULL) {
        return (TaskActionResult){TASK_STEP_FAILED, TASK_FAILURE_PROTOCOL};
    }

    TaskActionResult result = {TASK_STEP_FAILED, TASK_FAILURE_PROTOCOL};
    if (manager->active_action == TASK_ACTION_FOLLOW_TAPE) {
        result = follow_tape_action_update(&manager->follow_tape, now_ms);
    } else if (is_alignment_action(manager->active_action)) {
        result =
            tape_alignment_action_update(&manager->tape_alignment, now_ms);
    }
    if (task_step_status_is_terminal(result.status)) {
        manager->active_action = TASK_ACTION_COUNT;
    }
    return result;
}

// Cancels the selected action, using now_ms only to match the executor contract.
static void executor_cancel(void *context, uint32_t now_ms) {
    (void)now_ms;
    DrivetrainManager *manager = (DrivetrainManager *)context;
    if (manager == NULL) return;

    if (manager->active_action == TASK_ACTION_FOLLOW_TAPE) {
        follow_tape_action_cancel(&manager->follow_tape);
    } else if (is_alignment_action(manager->active_action)) {
        tape_alignment_action_cancel(&manager->tape_alignment);
    }
    manager->active_action = TASK_ACTION_COUNT;
}

// Wraps this manager in the generic interface required by TaskCoordinator.
TaskActionExecutor drivetrain_manager_executor(DrivetrainManager *manager) {
    if (manager == NULL) return (TaskActionExecutor){0};
    return (TaskActionExecutor){
        .context = manager,
        .start = executor_start,
        .update = executor_update,
        .cancel = executor_cancel,
    };
}

// Routes manual success injection to whichever action is currently active.
bool drivetrain_manager_report_succeeded(DrivetrainManager *manager) {
    if (manager == NULL) return false;
    if (manager->active_action == TASK_ACTION_FOLLOW_TAPE) {
        return follow_tape_action_report_succeeded(&manager->follow_tape);
    }
    return is_alignment_action(manager->active_action)
               ? tape_alignment_action_report_succeeded(
                     &manager->tape_alignment)
               : false;
}

// Routes a non-NONE manual failure reason to the currently active action.
bool drivetrain_manager_report_failed(DrivetrainManager *manager,
                                       TaskFailure failure) {
    if (manager == NULL) return false;
    if (manager->active_action == TASK_ACTION_FOLLOW_TAPE) {
        return follow_tape_action_report_failed(&manager->follow_tape,
                                                failure);
    }
    return is_alignment_action(manager->active_action)
               ? tape_alignment_action_report_failed(
                     &manager->tape_alignment, failure)
               : false;
}
