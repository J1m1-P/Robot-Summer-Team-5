/**
 * @file pick_up_block_action.cpp
 * @brief Implements the discrete servo and stepper steps for tower pickup.
 *
 * The action owns statically allocated mechanism drivers, applies one physical
 * command in start(), and polls only completion state in update(). This keeps
 * every pickup step nonblocking and independently sequenced by the coordinator.
 */
#include "task/actions/pick_up_block_action.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "config/servo_config.h"
#include "config/stepper_config.h"
#include "drivers/servo_driver.h"
#include "drivers/stepper_driver.h"

namespace {

struct PickupHardware {
    ServoDriver rotate; /**< Rotates the claw between front and down. */
    ServoDriver left; /**< Left small-claw servo. */
    ServoDriver middle; /**< Middle small-claw servo. */
    ServoDriver right; /**< Right small-claw servo. */
    StepperDriver tower_x; /**< Horizontal tower positioning axis. */
    StepperDriver tower_z; /**< Vertical tower lifting axis. */
};

// One hardware instance is shared across sequential pickup action commands.
PickupHardware hardware = {};

// Returns true for every discrete action implemented by this module.
bool pickup_action(TaskAction action) {
    return action == TASK_ACTION_PICK_UP_BLOCK ||
           action == TASK_ACTION_POSITION_TOWER_X ||
           action == TASK_ACTION_OPEN_TOWER_CLAWS ||
           action == TASK_ACTION_TOWER_FACE_DOWN ||
           action == TASK_ACTION_LOWER_TOWER ||
           action == TASK_ACTION_CLOSE_TOWER_CLAWS ||
           action == TASK_ACTION_RAISE_TOWER ||
           action == TASK_ACTION_TOWER_FACE_FRONT;
}

// Commands all three small claw servos to the same configured endpoint.
void set_claws(ServoPosition position) {
    servo_set_position(&hardware.left, position);
    servo_set_position(&hardware.middle, position);
    servo_set_position(&hardware.right, position);
}

// Identifies commands completed by elapsed settling time rather than a stepper.
bool servo_step(TaskAction action) {
    return action == TASK_ACTION_OPEN_TOWER_CLAWS ||
           action == TASK_ACTION_TOWER_FACE_DOWN ||
           action == TASK_ACTION_CLOSE_TOWER_CLAWS ||
           action == TASK_ACTION_TOWER_FACE_FRONT;
}

}  // namespace

// Initializes every mechanism driver and records whether the full set is usable.
void pick_up_block_action_init(PickUpBlockAction *action) {
    if (action == NULL) return;
    memset(action, 0, sizeof(*action));
    action->active_action = TASK_ACTION_COUNT;
    action->hardware_ready =
        servo_init(&hardware.rotate, towerRotateServoConfig) == ESP_OK &&
        servo_init(&hardware.left, towerLeftServoConfig) == ESP_OK &&
        servo_init(&hardware.middle, towerMiddleServoConfig) == ESP_OK &&
        servo_init(&hardware.right, towerRightServoConfig) == ESP_OK &&
        stepper_init(&hardware.tower_x, towerXConfig) == ESP_OK &&
        stepper_init(&hardware.tower_z, towerZConfig) == ESP_OK;
    action->result.status = TASK_STEP_NOT_STARTED;
}

// Applies one command and stores amount/settling parameters for update().
bool pick_up_block_action_start(PickUpBlockAction *action,
                                const TaskStepCommand *command,
                                uint32_t now_ms) {
    if (action == NULL || command == NULL ||
        !pickup_action(command->action) ||
        action->result.status == TASK_STEP_RUNNING ||
        !action->hardware_ready ||
        !isfinite(command->parameters.amount)) {
        return false;
    }

    action->active_action = command->action;
    action->parameters = command->parameters;
    action->started_ms = now_ms;
    action->result =
        (TaskActionResult){TASK_STEP_RUNNING, TASK_FAILURE_NONE};

    // Tower picking 4: adjust tower X by the requested signed metres.
    if (command->action == TASK_ACTION_POSITION_TOWER_X) {
        stepper_move_distanceMM(&hardware.tower_x,
                                command->parameters.amount * 1000.0f);
    }
    // Tower picking 5: open all three small claw servos.
    else if (command->action == TASK_ACTION_OPEN_TOWER_CLAWS) {
        set_claws(SERVO_POSITION_A);
    }
    // Tower picking 6: face the tower claw down (horizontal).
    else if (command->action == TASK_ACTION_TOWER_FACE_DOWN) {
        servo_set_position(&hardware.rotate, SERVO_POSITION_A);
    }
    // Tower picking 7: lower tower Z by the requested positive metres.
    else if (command->action == TASK_ACTION_LOWER_TOWER) {
        stepper_move_distanceMM(&hardware.tower_z,
                                command->parameters.amount * 1000.0f);
    }
    // Tower picking 8: close all three small claw servos.
    else if (command->action == TASK_ACTION_CLOSE_TOWER_CLAWS) {
        set_claws(SERVO_POSITION_B);
    }
    // Tower picking 9: raise tower Z by the requested positive metres.
    else if (command->action == TASK_ACTION_RAISE_TOWER) {
        stepper_move_distanceMM(&hardware.tower_z,
                                -command->parameters.amount * 1000.0f);
    }
    // Tower picking 10: face the tower claw front (vertical).
    else if (command->action == TASK_ACTION_TOWER_FACE_FRONT) {
        servo_set_position(&hardware.rotate, SERVO_POSITION_B);
    } else {
        action->result =
            (TaskActionResult){TASK_STEP_FAILED,
                               TASK_FAILURE_NOT_IMPLEMENTED};
    }
    return true;
}

// Advances steppers and detects stepper-stop or servo-settle completion.
void pick_up_block_action_update(PickUpBlockAction *action, uint32_t now_ms) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING) return;
    stepper_update(&hardware.tower_x);
    stepper_update(&hardware.tower_z);

    bool complete = false;
    if (action->active_action == TASK_ACTION_POSITION_TOWER_X) {
        complete = !stepper_is_moving(&hardware.tower_x);
    } else if (action->active_action == TASK_ACTION_LOWER_TOWER ||
               action->active_action == TASK_ACTION_RAISE_TOWER) {
        complete = !stepper_is_moving(&hardware.tower_z);
    } else if (servo_step(action->active_action)) {
        complete = (uint32_t)(now_ms - action->started_ms) >=
                   action->parameters.settle_ms;
    }
    if (complete) {
        action->result =
            (TaskActionResult){TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    }
}

// Stops moving axes and publishes a cancelled terminal result.
bool pick_up_block_action_cancel(PickUpBlockAction *action) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING) {
        return false;
    }
    stepper_stop(&hardware.tower_x);
    stepper_stop(&hardware.tower_z);
    action->result =
        (TaskActionResult){TASK_STEP_CANCELLED, TASK_FAILURE_NONE};
    return true;
}

// Allows a supervised test harness to force successful completion.
bool pick_up_block_action_report_succeeded(PickUpBlockAction *action) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING) {
        return false;
    }
    action->result =
        (TaskActionResult){TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    return true;
}

// Allows a supervised test harness to inject a mechanism failure.
bool pick_up_block_action_report_failed(PickUpBlockAction *action,
                                        TaskFailure failure) {
    if (action == NULL || action->result.status != TASK_STEP_RUNNING ||
        failure == TASK_FAILURE_NONE) {
        return false;
    }
    action->result = (TaskActionResult){TASK_STEP_FAILED, failure};
    return true;
}
