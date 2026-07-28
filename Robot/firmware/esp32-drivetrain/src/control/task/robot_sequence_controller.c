/* Implements the ordered drivetrain and arm action sequence. */
#include "control/task/robot_sequence_controller.h"

#include <stdio.h>

#include "esp32-hal.h"
#include <robot_common/command_packet.h>
#include <robot_common/pi_action_packet.h>
#include <robot_common/status_packet.h>

static const uint32_t kActionTimeoutMs = 15000;

// Replace this conversion after measuring camera steering on the robot.
#define PLACEHOLDER_VISION_ERROR_TO_DEGREES 45.0f

typedef enum {
    ROBOT_STEP_MOVEMENT = 0,
    ROBOT_STEP_ARM,
    ROBOT_STEP_PI_SCAN,
    ROBOT_STEP_SCAN_ROTATION,
} RobotStepType;

typedef struct {
    RobotStepType type;
    union {
        MovementAction movement;
        CommandOpcode arm;
    } action;
    float action_value;
} RobotSequenceStep;

// Actual Robot Sequence (To be finished).
static const RobotSequenceStep kRobotSequence[] = {
    // Scan at start
    //{ROBOT_STEP_PI_SCAN, {.arm = CMD_PI_SCAN_TELETUBBIES}, 0.0f},
    
    // Search checkpoint 1: tape follow, stop, and ask the Pi to scan.
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE},4.8f},
    // {ROBOT_STEP_PI_SCAN, {.arm = CMD_PI_SCAN_TELETUBBIES}, 0.0f},
    // {ROBOT_STEP_SCAN_ROTATION, {.movement = MOVEMENT_ACTION_ROTATE}, 0.0f},

    // // Search checkpoint 2: tape follow, stop, and ask the Pi to scan.
    // // Each scan sets the angle of the following rotation step.
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE},
    //  PLACEHOLDER_SCAN_DISTANCE_M},
    // {ROBOT_STEP_PI_SCAN, {.arm = CMD_PI_SCAN_TELETUBBIES}, 0.0f},
    // {ROBOT_STEP_SCAN_ROTATION, {.movement = MOVEMENT_ACTION_ROTATE}, 0.0f},

    // // Search checkpoint 3: tape follow, stop, and ask the Pi to scan.
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE},
    //  PLACEHOLDER_SCAN_DISTANCE_M},
    // {ROBOT_STEP_PI_SCAN, {.arm = CMD_PI_SCAN_TELETUBBIES}, 0.0f},
    // {ROBOT_STEP_SCAN_ROTATION, {.movement = MOVEMENT_ACTION_ROTATE}, 0.0f},

    // // Continue to tape follow
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE},
    //  PLACEHOLDER_SCAN_DISTANCE_M},

    // // Rotate to follow tape on the side to tower pickup, then align
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE_CW_UNTIL_TAPE_ALIGNED}, 90.0f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE}, 90.0f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_TOWER},
     PLACEHOLDER_SCAN_DISTANCE_M},

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

static bool step_is_movement(RobotStepType type) {
    return type == ROBOT_STEP_MOVEMENT ||
           type == ROBOT_STEP_SCAN_ROTATION;
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

static void advance_sequence(
    RobotSequenceController *controller,
    uint32_t now_ms);

static bool service_pi_report(
    RobotSequenceController *controller,
    const PacketFrame *frame) {
    // Decode the report received from the Pi.
    PiReportPacket report = {0};
    if (pi_report_packet_decode(frame, &report) != ESP_OK) return false;

    // Ignore a stale Pi report unless the sequence is waiting for a scan.
    if (kRobotSequence[controller->current_step].type !=
        ROBOT_STEP_PI_SCAN) {
        return false;
    }

    // Reports are repeated for reliability. Process each request only once.
    if (controller->last_pi_request_id == report.request_id) {
        return false;
    }
    controller->last_pi_request_id = report.request_id;

    // A miss still completes the scan; its programmed rotation becomes zero.
    if (report.result == PI_RESULT_NOT_FOUND) {
        controller->scan_rotation_degrees = 0.0f;
        printf("# Pi scan: no Teletubby detected\n");
        return true;
    }

    // Link, camera, and request failures stop the sequence.
    if (report.result != PI_RESULT_OK) {
        enter_fault(controller, "Pi scan failed", ESP_FAIL);
        return false;
    }

    // Positive camera error means the target is right of center. Drivetrain
    // rotation is positive counterclockwise, so rightward error becomes a
    // negative placeholder angle.
    controller->scan_rotation_degrees =
        -report.horizontal_error * PLACEHOLDER_VISION_ERROR_TO_DEGREES;
    printf(
        "# Pi scan: target %u detected at error %.3f; "
        "rotating %.1f degrees (placeholder)\n",
        (unsigned)report.target_id,
        report.horizontal_error,
        controller->scan_rotation_degrees);
    return true;
}

// Processes one already-dequeued frame from arm_uart. Callers own reading
// arm_uart (see comm/pose_service.h) since it's shared with odometry frames.
void robot_sequence_controller_handle_frame(
    RobotSequenceController *controller,
    const PacketFrame *frame,
    uint32_t now_ms) {
    if (controller == NULL || frame == NULL || !controller->running) return;

    bool step_complete = false;

    if (pi_report_packet_is(frame)) {
        step_complete = service_pi_report(controller, frame);
    } else if (status_packet_is(frame)) {
        StatusPacket status = {0};
        if (status_packet_decode(frame, &status) != ESP_OK) return;
        if (status.code == STATUS_FAULT) {
            enter_fault(controller, "arm reported a fault", ESP_FAIL);
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
                now_ms);
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

static esp_err_t start_robot_step(
    RobotSequenceController *controller,
    size_t step_index,
    uint32_t now_ms) {
    const RobotSequenceStep *step = &kRobotSequence[step_index];
    esp_err_t error = ESP_OK;

    if (step_is_movement(step->type)) {
        const float action_value =
            step->type == ROBOT_STEP_SCAN_ROTATION
                ? controller->scan_rotation_degrees
                : step->action_value;
        error = movement_action_controller_init(
            &controller->movement_action_controller,
            step->action.movement,
            action_value);
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
    if (controller->waiting_for_arm_ready) return;

    const RobotSequenceStep *step =
        &kRobotSequence[controller->current_step];
    if (step_is_movement(step->type)) {
        if (movement_action_controller_update(
                &controller->movement_action_controller)) {
            advance_sequence(controller, now_ms);
        } else {
            // false is always a terminal failure (no repeats)
            enter_fault(controller, "robot step failed", ESP_FAIL);
        }
        return;
    }

    if (deadline_reached(now_ms, controller->step_deadline_ms)) {
        enter_fault(controller, "robot step timed out", ESP_ERR_TIMEOUT);
    }
}
