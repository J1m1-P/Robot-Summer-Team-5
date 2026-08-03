/* Implements the ordered drivetrain and arm action sequence. */
#include "control/task/robot_sequence_controller.h"

#include <math.h>
#include <stdio.h>

#include "esp32-hal.h"
#include <robot_common/command_packet.h>
#include <robot_common/odometry_packet.h>
#include <robot_common/status_packet.h>

static const uint32_t kActionTimeoutMs = 15000;

typedef enum {
    ROBOT_STEP_MOVEMENT = 0,
    ROBOT_STEP_ARM,
} RobotStepType;

typedef struct {
    RobotStepType type;
    union {
        MovementAction movement;
        CommandOpcode arm;
    } action;
    float action_value;
} RobotSequenceStep;

// Actual Robot Sequence -- being redesigned from scratch. Add
// {ROBOT_STEP_MOVEMENT, {.movement = ...}, distance_or_angle} and
// {ROBOT_STEP_ARM, {.arm = ...}, value} entries here in the order the robot
// should run them.
static const RobotSequenceStep kRobotSequence[] = {
};

static const size_t kRobotSequenceLength =
    sizeof(kRobotSequence) / sizeof(kRobotSequence[0]);

static bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms) {
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool step_is_movement(RobotStepType type) {
    return type == ROBOT_STEP_MOVEMENT;
}

static bool controller_is_valid(const RobotSequenceController *controller) {
    return controller != NULL &&
           controller->pose_tracker != NULL &&
           controller->drivetrain != NULL &&
           controller->arm_uart != NULL &&
           controller->odometry_link != NULL;
}

static void enter_fault(
    RobotSequenceController *controller,
    const char *reason,
    esp_err_t error) {
    controller->running = false;
    printf(
        "# Robot sequence FAULT at step %u: %s (%s)\n",
        (unsigned)controller->current_step,
        reason,
        esp_err_to_name(error));
}

static esp_err_t start_robot_step(
    RobotSequenceController *controller,
    size_t step_index,
    uint32_t now_ms,
    float distance_override_m);

static void advance_sequence(
    RobotSequenceController *controller,
    uint32_t now_ms);

// -------------------------- Incoming UART logic --------------------------

// Selects the sequence logic for one non-odometry packet.
static void handle_sequence_frame(
    RobotSequenceController *controller,
    const PacketFrame *frame,
    uint32_t now_ms) {
    if (controller == NULL || frame == NULL || !controller->running) return;

    bool step_complete = false;

    if (status_packet_is(frame)) {
        StatusPacket status = {0};
        if (status_packet_decode(frame, &status) != ESP_OK) return;
        if (status.code == STATUS_FAULT) {
            enter_fault(controller, "arm reported a fault", ESP_FAIL);
            return;
        }

        if (status.code == STATUS_LOCATOR_CONTACT) {
            controller->locator_contact_pending = true;
            movement_action_controller_notify_locator_contact(
                &controller->movement_action_controller);
            return;
        }

        if (controller->waiting_for_arm_ready) {
            if (status.code != STATUS_ACTION_COMPLETE ||
                status.detail != STATUS_DETAIL_NONE) {
                return;
            }

            controller->waiting_for_arm_ready = false;
            const esp_err_t start_error = start_robot_step(
                controller,
                controller->current_step,
                now_ms,
                NAN);
            if (start_error != ESP_OK) {
                enter_fault(
                    controller,
                    "failed to start first robot step",
                    start_error);
            } else {
                printf("# Arm ready; robot sequence started\n");
            }
            return;
        }

        if (status.code != STATUS_ACTION_COMPLETE) return;

        const RobotSequenceStep *step =
            &kRobotSequence[controller->current_step];
        step_complete = step->type == ROBOT_STEP_ARM &&
            status.detail ==
                (uint8_t)arm_action_status_detail(step->action.arm);
    } else {
        return;
    }

    if (step_complete) advance_sequence(controller, now_ms);
}

// Drains every currently queued packet. Odometry and sequence messages share
// one UART, so routing happens immediately after each packet is dequeued.
static esp_err_t receive_arm_uart(
    RobotSequenceController *controller,
    uint32_t now_ms) {
    esp_err_t error = uart_link_update(controller->arm_uart);
    if (error != ESP_OK) return error;

    while (uart_link_has_packet(controller->arm_uart)) {
        PacketFrame frame = {0};
        error = uart_link_take_packet(controller->arm_uart, &frame);
        if (error != ESP_OK) return error;

        if (odometry_packet_is(&frame)) {
            odometry_link_ingest(controller->odometry_link, &frame);
        } else {
            handle_sequence_frame(controller, &frame, now_ms);
        }
    }
    return ESP_OK;
}

static esp_err_t update_pose(RobotSequenceController *controller) {
    const DrivetrainWheelCounts wheel_counts =
        drivetrain_get_wheel_counts(controller->drivetrain);
    const OdometryPacket *optical_packet =
        controller->odometry_link->has_packet
            ? &controller->odometry_link->latest
            : NULL;
    return pose_tracker_update(
        controller->pose_tracker, &wheel_counts, optical_packet);
}

static esp_err_t service_inputs(
    RobotSequenceController *controller,
    uint32_t now_ms) {
    const esp_err_t uart_error = receive_arm_uart(controller, now_ms);
    return uart_error == ESP_OK ? update_pose(controller) : uart_error;
}

// -------------------------- Outgoing UART logic --------------------------

static esp_err_t send_arm_command(
    RobotSequenceController *controller,
    CommandOpcode opcode,
    float value) {
    const CommandPacket command = {
        .opcode = opcode,
        .value = value,
    };
    return command_packet_send(controller->arm_uart, &command);
}

// --------------------------- Sequence logic ------------------------------

static esp_err_t start_robot_step(
    RobotSequenceController *controller,
    size_t step_index,
    uint32_t now_ms,
    float distance_override_m) {
    if (step_index >= kRobotSequenceLength) {
        // kRobotSequence is currently empty (sequence being redesigned) --
        // nothing to run.
        controller->running = false;
        printf("# Robot sequence complete (no steps defined)\n");
        return ESP_OK;
    }

    const RobotSequenceStep *step = &kRobotSequence[step_index];
    esp_err_t error = ESP_OK;

    if (step->type == ROBOT_STEP_MOVEMENT) {
        const float distance_m = isnan(distance_override_m)
            ? step->action_value
            : distance_override_m;
        error = movement_action_controller_init(
            &controller->movement_action_controller,
            step->action.movement,
            distance_m);
        if (error == ESP_OK &&
            step->action.movement ==
                MOVEMENT_ACTION_GO_BACKWARD_UNTIL_LOCATOR &&
            controller->locator_contact_pending) {
            movement_action_controller_notify_locator_contact(
                &controller->movement_action_controller);
            controller->locator_contact_pending = false;
        }
    } else {
        if (step->action.arm == CMD_TOWER_EXTEND_LOCATOR) {
            controller->locator_contact_pending = false;
        }
        error = send_arm_command(
            controller, step->action.arm, step->action_value);
    }
    if (error != ESP_OK) return error;

    controller->step_deadline_ms = now_ms + kActionTimeoutMs;
    controller->running = true;
    return ESP_OK;
}

static void advance_sequence(
    RobotSequenceController *controller,
    uint32_t now_ms) {
    ++controller->current_step;

    if (controller->current_step >= kRobotSequenceLength) {
        controller->running = false;
        printf("# Robot sequence complete\n");
        return;
    }

    const esp_err_t error = start_robot_step(
        controller, controller->current_step, now_ms, NAN);
    if (error != ESP_OK) {
        enter_fault(controller, "failed to start next robot step", error);
    }
}

esp_err_t robot_sequence_controller_init(
    RobotSequenceController *controller,
    PoseTracker *pose_tracker,
    Drivetrain *drivetrain,
    UartLink *arm_uart,
    Pmw3610OdometryLink *odometry_link) {
    if (controller == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *controller = (RobotSequenceController){0};
    controller->pose_tracker = pose_tracker;
    controller->drivetrain = drivetrain;
    controller->arm_uart = arm_uart;
    controller->odometry_link = odometry_link;
    if (!controller_is_valid(controller)) {
        *controller = (RobotSequenceController){0};
        return ESP_ERR_INVALID_ARG;
    }
    controller->waiting_for_arm_ready = true;
    controller->running = true;
    printf("# Waiting for arm controller\n");
    return ESP_OK;
}

esp_err_t robot_sequence_controller_start(
    RobotSequenceController *controller,
    uint32_t now_ms) {
    if (controller == NULL || !controller->running) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!controller->waiting_for_arm_ready) return ESP_ERR_INVALID_STATE;

    controller->waiting_for_arm_ready = false;
    const esp_err_t error = start_robot_step(
        controller, controller->current_step, now_ms, NAN);
    if (error != ESP_OK) controller->waiting_for_arm_ready = true;
    return error;
}

esp_err_t robot_sequence_controller_update(
    RobotSequenceController *controller,
    uint32_t now_ms) {
    if (!controller_is_valid(controller)) return ESP_ERR_INVALID_ARG;

    // Always service communication and pose, even after the sequence stops.
    const esp_err_t service_error = service_inputs(controller, now_ms);
    if (service_error != ESP_OK) return service_error;

    // A blocking movement calls this function from its own control loop.
    // That nested call services inputs above, then stops here.
    if (!controller->running ||
        controller->waiting_for_arm_ready ||
        controller->updating_movement) {
        return ESP_OK;
    }

    // Choose the update logic for the active step.
    const RobotSequenceStep *step =
        &kRobotSequence[controller->current_step];
    if (step_is_movement(step->type)) {
        controller->updating_movement = true;
        const bool succeeded = movement_action_controller_update(
            &controller->movement_action_controller);
        controller->updating_movement = false;

        if (succeeded) {
            advance_sequence(controller, now_ms);
        } else {
            // false is always a terminal failure (no repeats)
            enter_fault(controller, "robot step failed", ESP_FAIL);
        }
        return ESP_OK;
    }

    if (deadline_reached(now_ms, controller->step_deadline_ms)) {
        enter_fault(controller, "robot step timed out", ESP_ERR_TIMEOUT);
    }
    return ESP_OK;
}
