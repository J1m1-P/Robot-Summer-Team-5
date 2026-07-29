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

// Distance driven to each search checkpoint. Not required to be equal --
// replace each with a measured distance for its own tape section.
#define PLACEHOLDER_SCAN_DISTANCE_1_M 1.2f
#define PLACEHOLDER_SCAN_DISTANCE_2_M 1.2f
#define PLACEHOLDER_SCAN_DISTANCE_3_M 1.2f

// One teletubby search checkpoint: drive forward along the front tape by
// distance_m, then run the scan/rotate/flash loop (see service_pi_align).
// step_is_checkpoint() treats every pair this macro expands to as skippable
// once PI_RESULT_ALL_FOUND arrives mid-search.
#define SEARCH_CHECKPOINT(distance_m) \
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE}, \
     (distance_m)}, \
    {ROBOT_STEP_PI_ALIGN, {0}, 0.0f}

// When a scan comes back without a usable detection (nothing seen, camera
// hiccup, Pi didn't answer in time, link error) after we've already rotated
// this attempt, assume the last rotation overshot and turn back this
// fraction of it before trying again, rather than giving up outright.
#define NOT_FOUND_RECOVERY_FACTOR 0.5f

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
    // ════════════════════════════════════════════════════════════════════
    // TELETUBBY SEARCH — retract the locator and move the tower arm to a
    // safe search pose, then run three search checkpoints (tape-follow +
    // scan/align). See SEARCH_CHECKPOINT() above and service_pi_align()
    // below for what each checkpoint actually does.
    // ════════════════════════════════════════════════════════════════════
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_RETRACT_LOCATOR}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_OPEN_ALL_CLAWS}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_ROTATE_HORIZONTAL}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, 0.50f},

    SEARCH_CHECKPOINT(PLACEHOLDER_SCAN_DISTANCE_1_M),
    SEARCH_CHECKPOINT(PLACEHOLDER_SCAN_DISTANCE_2_M),
    SEARCH_CHECKPOINT(PLACEHOLDER_SCAN_DISTANCE_3_M),
    // ════════════════════════════════════════════════════════════════════
    // END TELETUBBY SEARCH
    // ════════════════════════════════════════════════════════════════════

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

// Starts a rotation as part of aligning and blocks until it's done (matching
// movement_action_controller_update's run-to-completion contract). Remembers
// the commanded angle for NOT_FOUND recovery. Used for real detections,
// REPOSITION/ALL_FOUND corrections, and recovery rotations alike. Returns
// false (and faults the sequence) only if the rotation itself fails to start
// or complete.
static bool run_align_rotation(
    RobotSequenceController *controller,
    float rotation_degrees,
    const char *fault_reason) {
    const esp_err_t init_error = movement_action_controller_init(
        &controller->movement_action_controller,
        MOVEMENT_ACTION_ROTATE,
        rotation_degrees);
    if (init_error != ESP_OK) {
        enter_fault(controller, fault_reason, init_error);
        return false;
    }
    controller->last_rotation_degrees = rotation_degrees;
    if (!movement_action_controller_update(
            &controller->movement_action_controller)) {
        enter_fault(controller, fault_reason, ESP_FAIL);
        return false;
    }
    return true;
}

// A search checkpoint is exactly the {FRONT_TAPE_FOLLOW_DISTANCE, PI_ALIGN}
// pair kRobotSequence repeats for each of the three search spots. Only these
// steps are skippable once every target is found -- the movement after the
// last checkpoint (rotate to the tower, side-tape-follow, strafe align, ...)
// is real navigation, not searching, and must still run.
static bool step_is_checkpoint(const RobotSequenceStep *step) {
    return step->type == ROBOT_STEP_PI_ALIGN ||
        (step->type == ROBOT_STEP_MOVEMENT &&
         step->action.movement == MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE);
}

// Skips past any remaining checkpoint steps straight to whatever comes after
// (the navigation toward Tower pickup). Lands one step early and lets
// advance_sequence's normal ++ do the rest, reusing the same start_robot_step
// path as an ordinary advance.
static bool skip_remaining_checkpoints(RobotSequenceController *controller) {
    size_t skip_to = controller->current_step + 1;
    while (skip_to < kRobotSequenceLength &&
           step_is_checkpoint(&kRobotSequence[skip_to])) {
        ++skip_to;
    }
    printf(
        "# Pi align: all targets found, skipping ahead to step %u\n",
        (unsigned)skip_to);
    controller->current_step = skip_to - 1;
    return true;
}

// Services one PiReportPacket for the current ROBOT_STEP_PI_ALIGN step. Each
// rotation this drives is a single blocking call
// (movement_action_controller_update() runs it to completion before
// returning), so there is no cross-tick "waiting on a rotation" state to
// track -- rescan-or-skip is decided immediately after the rotation, in the
// same call. Returns true once the step should advance (centered, not found,
// out of attempts, or the rotation itself failed).
static bool service_pi_align(
    RobotSequenceController *controller,
    const PacketFrame *frame,
    uint32_t now_ms) {
    PiReportPacket report = {0};
    if (pi_report_packet_decode(frame, &report) != ESP_OK) return false;

    // Reports are repeated for reliability. Process each request only once.
    if (controller->last_pi_request_id == report.request_id) return false;
    controller->last_pi_request_id = report.request_id;

    if (report.result == PI_RESULT_ALL_FOUND) {
        // Nothing left to search for. Not a detection -- horizontal_error is
        // ignored here. Instead, undo chase_net_rotation_degrees: the sum of
        // every rotation THIS controller has actually commanded since the
        // chase started, not a magnitude reported by the Pi. The Pi only
        // ever measures; it never learns how far a rotation actually turned
        // out, and its wire report clamps horizontal_error to [-1, 1] before
        // scaling, which could silently truncate a multi-attempt chase's
        // true net rotation. Tracking it here removes both problems --
        // whatever we commanded is exactly what we know to reverse.
        const float rotation_degrees = -controller->chase_net_rotation_degrees;
        controller->chase_net_rotation_degrees = 0.0f;
        if (fabsf(rotation_degrees) < ALIGN_CENTERED_DEGREES) {
            return skip_remaining_checkpoints(controller);  // nothing meaningful to undo
        }
        printf(
            "# Pi align: all targets found; repositioning %.1f degrees "
            "before moving on\n",
            rotation_degrees);
        if (!run_align_rotation(
                controller, rotation_degrees,
                "failed final repositioning rotation")) {
            return false;
        }
        return skip_remaining_checkpoints(controller);
    }

    if (report.result == PI_RESULT_REPOSITION) {
        // Not a detection -- the Pi is asking for a correction rotation
        // (e.g. to expose a second target it saw before this chase started
        // turning). horizontal_error is ignored for the same reason as
        // PI_RESULT_ALL_FOUND above -- undo chase_net_rotation_degrees, the
        // ESP's own record of what it actually commanded, not a Pi-reported
        // magnitude. Don't charge it against the attempt budget, and give
        // whatever's found next a fresh budget of its own rather than
        // sharing what's left of this one.
        const float rotation_degrees = -controller->chase_net_rotation_degrees;
        controller->align_attempts = 0;
        controller->chase_net_rotation_degrees = 0.0f;
        printf(
            "# Pi align: repositioning %.1f degrees (fresh attempt budget)\n",
            rotation_degrees);
        if (fabsf(rotation_degrees) >= ALIGN_CENTERED_DEGREES &&
            !run_align_rotation(
                controller, rotation_degrees,
                "failed repositioning rotation")) {
            return false;
        }
        const esp_err_t scan_error = send_pi_scan(controller, now_ms);
        if (scan_error != ESP_OK) {
            enter_fault(controller, "failed to re-request Pi scan", scan_error);
        }
        return false;
    }

    if (report.result != PI_RESULT_OK) {
        // No usable detection this round: nothing seen, a camera hiccup, the
        // Pi didn't answer in time, or a link error. None of these mean the
        // arm/drivetrain link itself is broken -- Tower/Habitat steps never
        // touch the Pi at all -- so retry within the attempt budget instead
        // of aborting the whole sequence over a vision-side miss.
        //
        // Exception: a NOT_FOUND before we've ever rotated this attempt has
        // nothing to recover FROM -- that's a genuinely empty checkpoint, not
        // a lost target, so give up immediately rather than burn attempts
        // spinning in place.
        const bool nothing_to_recover =
            report.result == PI_RESULT_NOT_FOUND &&
            controller->last_rotation_degrees == 0.0f;
        if (nothing_to_recover) {
            printf("# Pi scan: no Teletubby detected\n");
            return true;
        }

        ++controller->align_attempts;
        if (controller->align_attempts >= ALIGN_MAX_ATTEMPTS) {
            printf(
                "# Pi align: giving up this checkpoint (result %u, out of "
                "attempts)\n",
                (unsigned)report.result);
            return true;
        }
        const float recovery_degrees =
            -controller->last_rotation_degrees * NOT_FOUND_RECOVERY_FACTOR;
        printf(
            "# Pi align: result %u; recovering %.1f degrees (attempt %u/%u)\n",
            (unsigned)report.result,
            recovery_degrees,
            (unsigned)controller->align_attempts,
            ALIGN_MAX_ATTEMPTS);
        controller->chase_net_rotation_degrees += recovery_degrees;
        if (!run_align_rotation(
                controller, recovery_degrees, "failed recovery rotation")) {
            return false;
        }
        const esp_err_t scan_error = send_pi_scan(controller, now_ms);
        if (scan_error != ESP_OK) {
            enter_fault(controller, "failed to re-request Pi scan", scan_error);
        }
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

    controller->chase_net_rotation_degrees += rotation_degrees;
    if (!run_align_rotation(
            controller, rotation_degrees, "alignment rotation failed")) {
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
        controller->last_rotation_degrees = 0.0f;
        controller->chase_net_rotation_degrees = 0.0f;
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
