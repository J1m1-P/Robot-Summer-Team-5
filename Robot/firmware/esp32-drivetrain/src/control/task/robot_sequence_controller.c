/* Implements the ordered drivetrain and arm action sequence. */
#include "control/task/robot_sequence_controller.h"

#include <stdio.h>

#include "esp32-hal.h"
#include <robot_common/command_packet.h>
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

// Actual Robot Sequence (To be finished)
static const RobotSequenceStep kRobotSequence[] = {
    // Tape Follow: From start to after the ramp
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_TAPE_FOLLOW_DISTANCE}, 1.0f},
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE_CW_UNTIL_TAPE_ALIGNED}, 0.0f},

    // Tower: Picking up the pieces
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_HOME}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_ROTATE_VERTICAL}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_OPEN_CLAW}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z_UP}, 0.50f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_ROTATE_HORIZONTAL}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z_DOWN}, 0.50f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_CLOSE_CLAW}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z_UP}, 0.50f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_ROTATE_VERTICAL}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z_UP}, 0.30f},

    // Habitat: Building actions (enable and tune when the sequence is ready)
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_HOME}, 0.0f},
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_OPEN_CLAWS}, 0.0f},
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_Z_UP}, 0.30f},
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_CLOSE_CLAWS}, 0.0f},
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_X_RIGHT}, 0.90f},
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_OPEN_LEFT_CLAW}, 0.0f},
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_OPEN_RIGHT_CLAW}, 0.0f},

    // Movement: From tower pieces to tower base
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_FORWARD}, 1.0f},
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE}, 90.0f},
};

static const size_t kRobotSequenceLength =
    sizeof(kRobotSequence) / sizeof(kRobotSequence[0]);

static bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms) {
    return (int32_t)(now_ms - deadline_ms) >= 0;
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
    uint32_t now_ms);

static bool service_arm_uart(
    RobotSequenceController *controller,
    uint32_t now_ms) {
    const esp_err_t update_error = uart_link_update(controller->arm_uart);
    if (update_error != ESP_OK) {
        enter_fault(controller, "arm UART update failed", update_error);
        return false;
    }

    PacketFrame frame = {0};
    if (uart_link_take_packet(controller->arm_uart, &frame) != ESP_OK ||
        !status_packet_is(&frame)) {
        return false;
    }

    StatusPacket status = {0};
    if (status_packet_decode(&frame, &status) != ESP_OK) return false;
    if (status.code == STATUS_FAULT) {
        enter_fault(controller, "arm reported a fault", ESP_FAIL);
        return false;
    }

    if (controller->waiting_for_arm_ready) {
        if (status.code != STATUS_ACTION_COMPLETE ||
            status.detail != STATUS_DETAIL_NONE) {
            return false;
        }

        controller->waiting_for_arm_ready = false;
        const esp_err_t start_error = start_robot_step(
            controller,
            controller->current_step,
            now_ms);
        if (start_error != ESP_OK) {
            enter_fault(
                controller,
                "failed to start first robot step",
                start_error);
        } else {
            printf("# Arm ready; robot sequence started\n");
        }
        return false;
    }

    if (status.code != STATUS_ACTION_COMPLETE) return false;

    const RobotSequenceStep *step =
        &kRobotSequence[controller->current_step];
    return step->type == ROBOT_STEP_ARM &&
           status.detail ==
               (uint8_t)arm_action_status_detail(step->action.arm);
}

static esp_err_t start_robot_step(
    RobotSequenceController *controller,
    size_t step_index,
    uint32_t now_ms) {
    const RobotSequenceStep *step = &kRobotSequence[step_index];
    esp_err_t error = ESP_OK;

    if (step->type == ROBOT_STEP_MOVEMENT) {
        error = movement_action_controller_init(
            &controller->movement_action_controller,
            step->action.movement,
            step->action_value);
    } else {
        const CommandPacket command = {
            .opcode = step->action.arm,
            .value = step->action_value,
        };
        error = command_packet_send(controller->arm_uart, &command);
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

    const esp_err_t error =
        start_robot_step(controller, controller->current_step, now_ms);
    if (error != ESP_OK) {
        enter_fault(controller, "failed to start next robot step", error);
    }
}

esp_err_t robot_sequence_controller_init(
    RobotSequenceController *controller,
    UartLink *arm_uart) {
    if (controller == NULL || arm_uart == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *controller = (RobotSequenceController){0};
    controller->arm_uart = arm_uart;
    controller->waiting_for_arm_ready = true;
    controller->running = true;
    printf("# Waiting for arm controller\n");
    return ESP_OK;
}

void robot_sequence_controller_update(
    RobotSequenceController *controller,
    uint32_t now_ms) {
    if (controller == NULL || !controller->running) return;

    if (controller->waiting_for_arm_ready) {
        (void)service_arm_uart(controller, now_ms);
        return;
    }

    const RobotSequenceStep *step =
        &kRobotSequence[controller->current_step];
    const bool step_complete = step->type == ROBOT_STEP_MOVEMENT
        ? movement_action_controller_update(
            &controller->movement_action_controller)
        : service_arm_uart(controller, now_ms);
    if (!controller->running) return;

    if (step_complete) {
        advance_sequence(controller, now_ms);
    } else if (deadline_reached(now_ms, controller->step_deadline_ms)) {
        enter_fault(controller, "robot step timed out", ESP_ERR_TIMEOUT);
    }
}
