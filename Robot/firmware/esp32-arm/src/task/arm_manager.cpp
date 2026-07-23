#include "task/arm_manager.h"

#include <stddef.h>
#include <string.h>

#include <robot_common/app_log.h>

#include "config/servo_config.h"
#include "config/stepper_config.h"
#include "drivers/servo_driver.h"
#include "drivers/stepper_driver.h"

namespace {

// Keeps physical mechanism actions from being accepted by the drivetrain-side executor.
bool action_is_arm_owned(TaskAction action) {
    return action == TASK_ACTION_PICK_UP_BLOCK ||
           action == TASK_ACTION_BUILD_TOWER;
}

// One physical motion within a sequence. A step commands at most one
// stepper (relative move) and/or one servo (named position), then waits
// settle_ms after the stepper stops moving before the sequence advances.
struct ArmSequenceStep {
    StepperDriver *stepper;
    float distance_mm;
    ServoDriver *servo;
    ServoPosition servo_position;
    uint32_t settle_ms;
};

struct ArmSequence {
    const ArmSequenceStep *steps;
    uint8_t count;
};

// Owns the arm's stepper/servo hardware. A single static instance because
// this firmware only ever runs one ArmManager; keeping it out of the
// public ArmManager struct is what lets task_server.c (plain C) include
// arm_manager.h without pulling in Arduino/ESP32Servo C++ types.
struct ArmMechanisms {
    StepperDriver tower_x{};
    StepperDriver tower_z{};
    StepperDriver habitat_x{};
    StepperDriver habitat_z{};
    ServoDriver habitat_left{};
    ServoDriver habitat_right{};
    ServoDriver tower_rotate{};
    ServoDriver tower_left{};
    ServoDriver tower_middle{};
    ServoDriver tower_right{};
    bool ready = false;
};

ArmMechanisms g_mechanisms;

// TODO: fill in the real stepper distances / gripper order once the
// mechanism's physical travel and grip sequence are known. Empty tables make
// arm_manager_start reject the action instead of silently reporting success
// for motion that never happened.
const ArmSequence kPickUpBlockSequence = {nullptr, 0};
const ArmSequence kBuildTowerSequence = {nullptr, 0};

bool sequence_for(TaskAction action, ArmSequence *out) {
    switch (action) {
        case TASK_ACTION_PICK_UP_BLOCK:
            *out = kPickUpBlockSequence;
            return true;
        case TASK_ACTION_BUILD_TOWER:
            *out = kBuildTowerSequence;
            return true;
        default:
            return false;
    }
}

bool elapsed_at_least(uint32_t now, uint32_t then, uint32_t duration) {
    return (uint32_t)(now - then) >= duration;
}

// Services every stepper regardless of which action (if any) is running;
// cheap and a no-op for any stepper that isn't currently moving.
void service_steppers() {
    stepper_update(&g_mechanisms.tower_x);
    stepper_update(&g_mechanisms.tower_z);
    stepper_update(&g_mechanisms.habitat_x);
    stepper_update(&g_mechanisms.habitat_z);
}

void stop_all_steppers() {
    stepper_stop(&g_mechanisms.tower_x);
    stepper_stop(&g_mechanisms.tower_z);
    stepper_stop(&g_mechanisms.habitat_x);
    stepper_stop(&g_mechanisms.habitat_z);
}

// Issues one sequence step's motion and starts its settle timer.
void begin_step(ArmManager *manager, const ArmSequenceStep &step,
                uint8_t step_index, uint32_t now_ms) {
    if (step.servo != nullptr) {
        servo_set_position(step.servo, step.servo_position);
    }
    if (step.stepper != nullptr) {
        stepper_move_distanceMM(step.stepper, step.distance_mm);
    }
    manager->current_step = step_index;
    manager->step_started_ms = now_ms;
}

}  // namespace

// Initializes every stepper/servo once. Continues (with mechanisms unready)
// if any hardware init fails so the rest of the arm still boots.
void arm_manager_init(ArmManager *manager) {
    if (manager == nullptr) return;
    memset(manager, 0, sizeof(*manager));
    manager->status = TASK_STEP_NOT_STARTED;
    manager->mechanisms = &g_mechanisms;

    esp_err_t error = stepper_init(&g_mechanisms.tower_x, towerXConfig);
    if (error == ESP_OK) {
        error = stepper_init(&g_mechanisms.tower_z, towerZConfig);
    }
    if (error == ESP_OK) {
        error = stepper_init(&g_mechanisms.habitat_x, habitatXConfig);
    }
    if (error == ESP_OK) {
        error = stepper_init(&g_mechanisms.habitat_z, habitatZConfig);
    }
    if (error == ESP_OK) {
        error = servo_init(&g_mechanisms.habitat_left, habitatLeftServoConfig);
    }
    if (error == ESP_OK) {
        error = servo_init(&g_mechanisms.habitat_right, habitatRightServoConfig);
    }
    if (error == ESP_OK) {
        error = servo_init(&g_mechanisms.tower_rotate, towerRotateServoConfig);
    }
    if (error == ESP_OK) {
        error = servo_init(&g_mechanisms.tower_left, towerLeftServoConfig);
    }
    if (error == ESP_OK) {
        error = servo_init(&g_mechanisms.tower_middle, towerMiddleServoConfig);
    }
    if (error == ESP_OK) {
        error = servo_init(&g_mechanisms.tower_right, towerRightServoConfig);
    }

    if (error != ESP_OK) {
        APP_LOGE(LOG_TAG_MAIN,
                 "Arm mechanism init failed; PICK_UP_BLOCK/BUILD_TOWER "
                 "disabled: %s",
                 esp_err_to_name(error));
        g_mechanisms.ready = false;
        return;
    }
    g_mechanisms.ready = true;
}

// Accepts one arm-owned command when no other arm action is active and the
// requested action has a non-empty sequence to run.
bool arm_manager_start(ArmManager *manager, const TaskStepCommand *command,
                       uint32_t now_ms) {
    if (manager == nullptr || command == nullptr ||
        manager->status == TASK_STEP_RUNNING ||
        !action_is_arm_owned(command->action)) {
        return false;
    }

    ArmSequence sequence = {nullptr, 0};
    if (!sequence_for(command->action, &sequence) || !g_mechanisms.ready ||
        sequence.count == 0U) {
        return false;
    }

    manager->active_action = command->action;
    manager->status = TASK_STEP_RUNNING;
    manager->failure = TASK_FAILURE_NONE;
    begin_step(manager, sequence.steps[0], 0, now_ms);
    return true;
}

// Services stepper motion every cycle, then advances the active sequence
// once its current step's stepper has stopped and settle_ms has elapsed.
void arm_manager_update(ArmManager *manager, uint32_t now_ms) {
    service_steppers();
    if (manager == nullptr || manager->status != TASK_STEP_RUNNING) return;

    ArmSequence sequence = {nullptr, 0};
    if (!sequence_for(manager->active_action, &sequence) ||
        manager->current_step >= sequence.count) {
        manager->status = TASK_STEP_FAILED;
        manager->failure = TASK_FAILURE_PROTOCOL;
        return;
    }

    const ArmSequenceStep &step = sequence.steps[manager->current_step];
    const bool motion_done =
        step.stepper == nullptr || !stepper_is_moving(step.stepper);
    if (!motion_done ||
        !elapsed_at_least(now_ms, manager->step_started_ms, step.settle_ms)) {
        return;
    }

    const uint8_t next_step = (uint8_t)(manager->current_step + 1U);
    if (next_step >= sequence.count) {
        manager->status = TASK_STEP_SUCCEEDED;
        manager->failure = TASK_FAILURE_NONE;
        return;
    }
    begin_step(manager, sequence.steps[next_step], next_step, now_ms);
}

// Stops every stepper immediately and publishes a cancelled result.
bool arm_manager_cancel(ArmManager *manager) {
    if (manager == nullptr || manager->status != TASK_STEP_RUNNING) return false;
    stop_all_steppers();
    manager->status = TASK_STEP_CANCELLED;
    manager->failure = TASK_FAILURE_NONE;
    return true;
}

// Publishes successful physical completion to the task server.
bool arm_manager_report_succeeded(ArmManager *manager) {
    if (manager == nullptr || manager->status != TASK_STEP_RUNNING) {
        return false;
    }
    manager->status = TASK_STEP_SUCCEEDED;
    manager->failure = TASK_FAILURE_NONE;
    return true;
}

// Publishes a nonzero physical failure, stops every stepper, and closes the action.
bool arm_manager_report_failed(ArmManager *manager, TaskFailure failure) {
    if (manager == nullptr || manager->status != TASK_STEP_RUNNING ||
        failure == TASK_FAILURE_NONE) {
        return false;
    }
    stop_all_steppers();
    manager->status = TASK_STEP_FAILED;
    manager->failure = failure;
    return true;
}

// Returns the current action result without transferring ownership of manager state.
TaskStepStatus arm_manager_get_status(const ArmManager *manager,
                                      TaskFailure *failure_out) {
    if (manager == nullptr) {
        if (failure_out != nullptr) *failure_out = TASK_FAILURE_PROTOCOL;
        return TASK_STEP_FAILED;
    }
    if (failure_out != nullptr) *failure_out = manager->failure;
    return manager->status;
}
