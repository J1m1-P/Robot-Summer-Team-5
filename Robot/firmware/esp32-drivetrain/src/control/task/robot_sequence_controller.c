/* Implements the ordered drivetrain and arm action sequence. */
#include "control/task/robot_sequence_controller.h"

#include <math.h>
#include <stdio.h>

#include "esp32-hal.h"
#include <robot_common/command_packet.h>
#include <robot_common/pi_action_packet.h>
#include <robot_common/status_packet.h>

static const uint32_t kActionTimeoutMs = 15000;

// Replace this conversion after measuring camera steering on the robot.
#define PLACEHOLDER_VISION_ERROR_TO_DEGREES 45.0f

// ROBOT_STEP_PI_ALIGN tuning. Each pass asks the Pi for one scan and, if the
// reported error is too large, rotates and scans again. ALIGN_CENTERED_DEGREES
// should stay close to the Pi's own ALIGN_THRESHOLD (in tubby_detector.py)
// converted through PLACEHOLDER_VISION_ERROR_TO_DEGREES, since that's the
// threshold the Pi itself uses to decide whether to flash on a given scan.
#define ALIGN_MAX_ATTEMPTS 4
#define ALIGN_CENTERED_DEGREES 3.5f

// Distance driven between search checkpoints. Replace after measuring the
// real tape sections.
#define PLACEHOLDER_SCAN_DISTANCE_M 1.2f

typedef enum {
    ROBOT_STEP_MOVEMENT = 0,
    ROBOT_STEP_ARM,
    ROBOT_STEP_PI_ALIGN,
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
//
// ROBOT_STEP_PI_ALIGN is one table entry per checkpoint but runs its own
// internal scan/rotate loop (see service_pi_align): scan, and if the target
// isn't centered, rotate and scan again, up to ALIGN_MAX_ATTEMPTS times. The
// robot is stationary during every scan, so whichever scan measures a
// centered error is the one where the Pi actually flashes -- no separate
// "aligned" signal is needed, the next scan IS the confirmation.
static const RobotSequenceStep kRobotSequence[] = {
    // Retract the locator
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_RETRACT_LOCATOR}, 0.0f},

    // Move tower arm from home position to safe idle position
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_OPEN_ALL_CLAWS}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_ROTATE_HORIZONTAL}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, 0.50f},

    // Search checkpoint 1: tape follow, then align on and flash a Teletubby.
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE},
     PLACEHOLDER_SCAN_DISTANCE_M},
    {ROBOT_STEP_PI_ALIGN, {0}, 0.0f},

    // Search checkpoint 2.
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE},
     PLACEHOLDER_SCAN_DISTANCE_M},
    {ROBOT_STEP_PI_ALIGN, {0}, 0.0f},

    // Search checkpoint 3.
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE},
     PLACEHOLDER_SCAN_DISTANCE_M},
    {ROBOT_STEP_PI_ALIGN, {0}, 0.0f},

    // Rotate to follow tape on the side to tower pickup, then align.
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE}, 90.0f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_TOWER}, 0.0f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_BACK_TAPE_STRAFE_ALIGN}, 0.0f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, -0.05f},

    // Tower: Picking up the pieces
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, -0.50f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_CLOSE_ALL_CLAWS}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, 0.20f},
    // TODO: Do following arm steps in parallel with movement to save time
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, 0.30f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_ROTATE_VERTICAL}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, 0.30f},

    // Move to the tower base
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_FORWARD_UNTIL_SIDE_TAPE}, 0.0f},
    // TODO: make locator extend during movement to save time
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_EXTEND_LOCATOR}, 0.0f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_TOWER}, 0.0f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_BACK_TAPE_STRAFE_ALIGN}, 0.0f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_BACKWARD_UNTIL_LOCATOR}, 0.0f},

    // Tower: Placing the pieces
    // TODO: tune Z distances
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, -0.30f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_OPEN_MIDDLE_CLAW}, 0.0f},    // drop middle piece
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, 0.50f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_X}, 0.68f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, -0.30f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_OPEN_LEFT_CLAW}, 0.0f},  // drop left piece
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, 0.50f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_X}, -1.36f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, -0.30f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_OPEN_RIGHT_CLAW}, 0.0f}, // drop right piece

    // Go back to main tape and put tower arm and locator in safe idle position
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_RETRACT_LOCATOR}, 0.0f},     // retract locator
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, -1.0f},                  // idle position for Z
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_FORWARD_UNTIL_SIDE_TAPE}, 0.0f},
    // TODO: do arm actions while driving to save time
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_ROTATE_HORIZONTAL}, 0.0f},   // rehome rotation
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_CLOSE_ALL_CLAWS}, 0.0f},     // close all claws
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_X}, 0.68f},                  // rehome X

    // Move to habitat tape to home
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_HABITAT}, 0.0f},

    // Habitat: Building actions (enable and tune when the sequence is ready)
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_HOME}, 0.0f},
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_OPEN_CLAWS}, 0.0f},
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_Z_UP}, 0.30f},
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_CLOSE_CLAWS}, 0.0f},
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_X_RIGHT}, 0.90f},
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_OPEN_LEFT_CLAW}, 0.0f},
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_OPEN_RIGHT_CLAW}, 0.0f},
};

static const size_t kRobotSequenceLength =
    sizeof(kRobotSequence) / sizeof(kRobotSequence[0]);

static bool deadline_reached(uint32_t now_ms, uint32_t deadline_ms) {
    return (int32_t)(now_ms - deadline_ms) >= 0;
}

static bool step_is_movement(RobotStepType type) {
    return type == ROBOT_STEP_MOVEMENT;
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

// Sends one CMD_PI_SCAN_TELETUBBIES and refreshes the step deadline. Used both
// to start a PI_ALIGN step and to re-scan after each alignment rotation.
static esp_err_t send_pi_scan(
    RobotSequenceController *controller,
    uint32_t now_ms) {
    const CommandPacket command = {
        .opcode = CMD_PI_SCAN_TELETUBBIES,
        .value = 0.0f,
    };
    const esp_err_t error = command_packet_send(controller->arm_uart, &command);
    if (error == ESP_OK) {
        controller->step_deadline_ms = now_ms + kActionTimeoutMs;
    }
    return error;
}

// Services one PiReportPacket for the current ROBOT_STEP_PI_ALIGN step: if
// the reported error is too large and attempts remain, rotate (blocking --
// movement_action_controller_update() runs the whole rotation to completion
// before returning) and re-request a scan; otherwise the step is done.
// Returns true once the step should advance (centered, not found, out of
// attempts, or the rotation itself failed).
static bool service_pi_align(
    RobotSequenceController *controller,
    const PacketFrame *frame,
    uint32_t now_ms) {
    PiReportPacket report = {0};
    if (pi_report_packet_decode(frame, &report) != ESP_OK) return false;

    // Reports are repeated for reliability. Process each request only once.
    if (controller->last_pi_request_id == report.request_id) return false;
    controller->last_pi_request_id = report.request_id;

    if (report.result == PI_RESULT_NOT_FOUND) {
        printf("# Pi scan: no Teletubby detected\n");
        return true;
    }
    if (report.result != PI_RESULT_OK) {
        enter_fault(controller, "Pi scan failed", ESP_FAIL);
        return false;
    }

    // Positive camera error means the target is right of center. Drivetrain
    // rotation is positive counterclockwise, so rightward error becomes a
    // negative angle.
    const float rotation_degrees =
        -report.horizontal_error * PLACEHOLDER_VISION_ERROR_TO_DEGREES;
    printf(
        "# Pi scan: target %u detected at error %.3f; %.1f degrees off "
        "(attempt %u/%u)\n",
        (unsigned)report.target_id,
        report.horizontal_error,
        rotation_degrees,
        (unsigned)controller->align_attempts + 1,
        ALIGN_MAX_ATTEMPTS);

    if (fabsf(rotation_degrees) < ALIGN_CENTERED_DEGREES) {
        return true;  // centered -> the Pi already flashed on this scan
    }
    ++controller->align_attempts;
    if (controller->align_attempts >= ALIGN_MAX_ATTEMPTS) {
        printf("# Pi align: out of attempts, moving on\n");
        return true;
    }

    const esp_err_t init_error = movement_action_controller_init(
        &controller->movement_action_controller,
        MOVEMENT_ACTION_ROTATE,
        rotation_degrees);
    if (init_error != ESP_OK) {
        enter_fault(controller, "failed to start alignment rotation", init_error);
        return false;
    }
    if (!movement_action_controller_update(&controller->movement_action_controller)) {
        enter_fault(controller, "alignment rotation failed", ESP_FAIL);
        return false;
    }
    const esp_err_t scan_error = send_pi_scan(controller, now_ms);
    if (scan_error != ESP_OK) {
        enter_fault(controller, "failed to re-request Pi scan", scan_error);
    }
    return false;
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
        if (kRobotSequence[controller->current_step].type != ROBOT_STEP_PI_ALIGN) {
            return;  // stale report for a step we've already left
        }
        step_complete = service_pi_align(controller, frame, now_ms);
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

    if (step->type == ROBOT_STEP_MOVEMENT) {
        error = movement_action_controller_init(
            &controller->movement_action_controller,
            step->action.movement,
            step->action_value);
    } else if (step->type == ROBOT_STEP_PI_ALIGN) {
        controller->align_attempts = 0;
        error = send_pi_scan(controller, now_ms);
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
