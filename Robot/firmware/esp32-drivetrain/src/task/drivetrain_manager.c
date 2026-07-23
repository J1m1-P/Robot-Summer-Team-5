#include "task/drivetrain_manager.h"

#include <stddef.h>
#include <string.h>

#include <robot_common/app_log.h>

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
    drivetrain_action_dispatcher_init(&manager->action_dispatcher);

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

    const DrivetrainActionHandler follow_tape =
        follow_tape_action_handler(&manager->follow_tape);
    const DrivetrainActionHandler tape_alignment =
        tape_alignment_action_handler(&manager->tape_alignment);
    if (!drivetrain_action_dispatcher_register(
            &manager->action_dispatcher, &follow_tape) ||
        !drivetrain_action_dispatcher_register(
            &manager->action_dispatcher, &tape_alignment)) {
        APP_LOGE(LOG_TAG_DRIVETRAIN,
                 "Drivetrain action registration failed");
        manager->tape_hardware_ready = false;
        return ESP_ERR_INVALID_STATE;
    }

    manager->tape_hardware_ready = true;
    return ESP_OK;
}

TaskActionExecutor drivetrain_manager_executor(DrivetrainManager *manager) {
    if (manager == NULL) return (TaskActionExecutor){0};
    return drivetrain_action_dispatcher_executor(
        &manager->action_dispatcher);
}

bool drivetrain_manager_report_succeeded(DrivetrainManager *manager) {
    if (manager == NULL) return false;
    return drivetrain_action_dispatcher_report_succeeded(
        &manager->action_dispatcher);
}

bool drivetrain_manager_report_failed(DrivetrainManager *manager,
                                      TaskFailure failure) {
    if (manager == NULL) return false;
    return drivetrain_action_dispatcher_report_failed(
        &manager->action_dispatcher, failure);
}
