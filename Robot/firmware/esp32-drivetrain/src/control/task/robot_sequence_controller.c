/* Implements the ordered drivetrain and arm action sequence. */
#include "control/task/robot_sequence_controller.h"

#include <math.h>
#include <stdio.h>
#include <stdint.h>

#include "esp32-hal.h"
#include <robot_common/command_packet.h>
#include <robot_common/odometry_packet.h>
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
    ROBOT_STEP_DELAY,
} RobotStepType;

typedef struct {
    RobotStepType type;
    union {
        MovementAction movement;
        CommandOpcode arm;
    } action;
    float action_value;
    float action_speed_mps;
} RobotSequenceStep;

static const RobotSequenceStep kRobotSequence[] = {
    /* Arm Homing Sequence. Use homing blocks and 
    position habitat claws just before the start line */
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_RETRACT_LOCATOR}, 0.0f}, // retract locator
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_OPEN_ALL_CLAWS}, 0.0f},  // open tower claws
    {ROBOT_STEP_ARM, {.arm = CMD_TOWER_ROTATE_HORIZONTAL}, 0.0f}, // horizontal tower position
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_OPEN_CLAWS}, 0.0f}, // open habitat claws
    //{ROBOT_STEP_ARM, {.arm = CMD_HABITAT_X}, 0.3}   // defend wheels from rocks
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, 0.4f}, // remove when new homers arrive

    /* Tape follow straight to rotate step. Remove when PI works*/
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_PX_TAPE_FOLLOW_DISTANCE}, 4.9f, 0.6f},

    // Teletubbies / Rock Path

    // Scan at start
    //{ROBOT_STEP_PI_SCAN, {.arm = CMD_PI_SCAN_TELETUBBIES}, 0.0f},
    
    // Search checkpoint 1: tape follow, stop, and ask the Pi to scan.
    //{ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_PX_TAPE_FOLLOW_DISTANCE}, 4.9f, 0.55f},
    // {ROBOT_STEP_PI_SCAN, {.arm = CMD_PI_SCAN_TELETUBBIES}, 0.0f},
    // {ROBOT_STEP_SCAN_ROTATION, {.movement = MOVEMENT_ACTION_ROTATE}, 0.0f},

    // // Search checkpoint 2: tape follow, stop, and ask the Pi to scan.
    // // Each scan sets the angle of the following rotation step.
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_PX_TAPE_FOLLOW_DISTANCE},
    //  PLACEHOLDER_SCAN_DISTANCE_M, 0.35f},
    // {ROBOT_STEP_PI_SCAN, {.arm = CMD_PI_SCAN_TELETUBBIES}, 0.0f},
    // {ROBOT_STEP_SCAN_ROTATION, {.movement = MOVEMENT_ACTION_ROTATE}, 0.0f},

    // // Search checkpoint 3: tape follow, stop, and ask the Pi to scan.
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_PX_TAPE_FOLLOW_DISTANCE},
    //  PLACEHOLDER_SCAN_DISTANCE_M, 0.35f},
    // {ROBOT_STEP_PI_SCAN, {.arm = CMD_PI_SCAN_TELETUBBIES}, 0.0f},
    // {ROBOT_STEP_SCAN_ROTATION, {.movement = MOVEMENT_ACTION_ROTATE}, 0.0f},

    // // Continue to tape follow
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_PX_TAPE_FOLLOW_DISTANCE},
    //  PLACEHOLDER_SCAN_DISTANCE_M, 0.35f},


    // /* Movement to tower pickup*/
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE_CW_UNTIL_SIDE_TAPE}, 120.0f, 2.0f}, // rotate until side tape
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_TOWER}, 0.0f, 0.15f}, // tape follow to pickup
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, -0.07f, 0.2f},  // forward/backward adjustment
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_Y_DISTANCE}, 0.035f, 0.2f},   // left/right adjustment

    // /* Picking up tower pieces*/ 
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, -0.40f},                 // lower claw
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_CLOSE_ALL_CLAWS}, 0.0f},     // close claws
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, 0.40f},                  
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_ROTATE_VERTICAL}, 0.0f},
    // // // TODO: Do following arm steps in parallel with movement to save time
    // //{ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, 0.30f},

    // // Move to the tower base
    // // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_Y_DISTANCE}, -0.02f, 0.35f},
    // // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, 0.065f, 0.2f},
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_PX_UNTIL_SIDE_TAPE}, 0.0f, 0.2f},
    // {ROBOT_STEP_DELAY, {0}, 1.0f},
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_PY_TAPE_FOLLOW_DISTANCE}, 0.12f, 0.4f},
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_TOWER}, 0.0f, 0.15f},
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_Y_DISTANCE}, 0.06f, 0.2f},
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_PX_DISTANCE}, 0.05f, 0.2f},
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_EXTEND_LOCATOR}, 0.0f},
    // {ROBOT_STEP_DELAY, {0}, 1.6f},
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_MX_UNTIL_LOCATOR}, 1.0f, 0.09f},
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_RETRACT_LOCATOR}, 0.0f},



    // /*Placing the tower pieces*/
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, -0.3f},                  // lower claw
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_OPEN_MIDDLE_CLAW}, 0.0f},    // drop middle piece
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, 1.0f},                   // raise claw
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_X}, 0.68f},                  // move X
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, -0.25f},                 // lower claw
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_OPEN_LEFT_CLAW}, 0.0f},      // drop left piece
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, 1.2f},                   // raise claw
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_X}, -0.68f},                 // move X
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_X}, -0.68f},                 // move X again
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, -0.35f},                 // lower claw
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_OPEN_RIGHT_CLAW}, 0.0f},     // drop right piece
    
    // Go back to main tape and put tower arm and locator in safe idle position

    /*------------------------ Habitat Code Below ------------------------*/
    
    // // TODO: do arm actions while driving to save time
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z}, -1.0f},                  // idle Z position
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_ROTATE_HORIZONTAL}, 0.0f},   // idle rotation
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_CLOSE_ALL_CLAWS}, 0.0f},     // close all claws
    // // TODO: do arm actions while driving to save time
    // {ROBOT_STEP_ARM, {.arm = CMD_TOWER_X}, 0.68f},                  // idle X position
    

    /*
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_PX_UNTIL_SIDE_TAPE}, 0.0f, 0.3f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE_CCW_UNTIL_FRONT_TAPE}, 160.0f, 1.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_Z}, 0.6f},                 // raise claw

    // Move to habitat tape to home
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_PX_TAPE_FOLLOW_DISTANCE}, 1.0f, 0.4f},
    */
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_Z}, 0.7f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_PX_UNTIL_SIDE_TAPE}, 0.0f, 0.2f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE_CCW_UNTIL_FRONT_TAPE}, 120.0f, 1.0f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_PX_TAPE_FOLLOW_DISTANCE}, 0.5f, 0.35f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_PX_TAPE_FOLLOW_UNTIL_ALL_CHANNELS_ON}, 0.0f, 0.2f},
    
    // Go off the tape and go for the habitats 1 and 2
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE}, 7.0f, 0.3f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, -0.06f, 0.2f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_PY_DISTANCE}, 0.19f, 0.2f}, 

    // Pick up the habitats 1 and 2
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_Z}, -0.68f},                 // lower claw
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, 0.04f, 0.05f}, 
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_CLOSE_CLAWS}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_Z}, 0.35f},                 // raise claw

    // Move to the habitat base from 1 and 2
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, -0.15f, 0.2f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE}, 90.0f, 1.0f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_MX_UNTIL_SIDE_TAPE}, 0.0f, 0.25f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, -0.03f, 0.2f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_PX_UNTIL_SIDE_TAPE}, 0.0f, 0.2f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_HABITAT_FRONT}, 0.0f, 0.15f}, 

    // PLACEDOWN Sequence 1
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE}, -1.0f, 1.0f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, 0.02f, 0.2f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_Y_DISTANCE}, -0.10f, 0.3f},
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_OPEN_LEFT_CLAW}, 0.0f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, -0.02f, 0.2f}, 

    // // PLACEDOWN Sequence 2
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_X}, 0.6},    
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, -0.01f, 0.15f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, 0.02f, 0.2f}, 
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_SEMI_CLOSE_LEFT_CLAW}, 0.0f}, 
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_X}, -1.25},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_Y_DISTANCE}, 0.097f, 0.35f},
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_OPEN_RIGHT_CLAW}, 0.0f}, 
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_OPEN_LEFT_CLAW}, 0.0f}, 

    // Turn the two habitats
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, -0.02f, 0.2f},
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_X}, 0.75},
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_X}, 0.75},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_Y_DISTANCE}, -0.085f, 0.15f},
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_SEMI_CLOSE_LEFT_CLAW}, 0.0f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, 0.02f, 0.2f},
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_X}, -0.7},
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_X}, -0.7},
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_OPEN_LEFT_CLAW}, 0.0f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, -0.02f, 0.2f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_Y_DISTANCE}, -0.06f, 0.4f},

    // Rehome the two steppers
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_X}, 0.55},
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_Z}, 0.33},

    // Go for habitats again
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE_CW_UNTIL_FRONT_TAPE}, 170.0f, 1.5f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_PX_TAPE_FOLLOW_DISTANCE}, 0.1f, 0.35f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_PX_TAPE_FOLLOW_UNTIL_ALL_CHANNELS_ON}, 0.0f, 0.1f},

    // Go off the tape and go for the habitats 3 and 4
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE}, 8.0f, 0.3f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, -0.06f, 0.2f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_PY_DISTANCE}, -0.19f, 0.2f}, 

    // Pick up the habitats 3 and 4
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_Z}, -0.58f},                 // lower claw
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, 0.06f, 0.05f}, 
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_CLOSE_CLAWS}, 0.0f},
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_Z}, 0.35f},                 // raise claw

    // Move to the habitat base from 3 and 4
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, -0.25f, 0.2f},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE}, 90.0f, 1.0f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_PX_UNTIL_SIDE_TAPE}, 0.0f, 0.2f},
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_X}, 0.95},
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_SIDE_TAPE_FOLLOW_UNTIL_HABITAT_FRONT}, 0.0f, 0.15f}, 

    // PLACEDOWN Sequence 3
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE}, -2.0f, 1.0f}, 
    {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, 0.02f, 0.15f}, 
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_Y_DISTANCE}, -0.12f, 0.17f},
    {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_OPEN_LEFT_CLAW}, 0.0f}, 

    
    // PLACEDOWN Sequence 4
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_X}, 0.6},    
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_SEMI_CLOSE_LEFT_CLAW}, 0.0f}, 
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_X}, -1.5},    
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_Y_DISTANCE}, 0.1f, 0.17f},
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_OPEN_RIGHT_CLAW}, 0.0f}, 
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_OPEN_LEFT_CLAW}, 0.0f}, 
     
    /*------------------------ Solar Panel Code Below ------------------------*/
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, -0.40f, 0.3f},   // move back to solar panel
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE}, 90.0f, 1.0f},    // rotate so spikes are aligned
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_Z}, -0.06f},                                   // move claw to position in Y
    // {ROBOT_STEP_ARM, {.arm = CMD_HABITAT_X}, -0.16f},                                   // move claw to position in X
    // {ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_X_DISTANCE}, 0.5f, 0.25f},   // move and remove cover
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

static bool step_is_delay(RobotStepType type) {
    return type == ROBOT_STEP_DELAY;
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
    uint32_t now_ms);

static void advance_sequence(
    RobotSequenceController *controller,
    uint32_t now_ms);

// -------------------------- Incoming UART logic --------------------------

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

// Selects the sequence logic for one non-odometry packet.
static void handle_sequence_frame(
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
    uint32_t now_ms) {
    const RobotSequenceStep *step = &kRobotSequence[step_index];
    esp_err_t error = ESP_OK;

    if (step_is_delay(step->type)) {
        if (!isfinite(step->action_value) || step->action_value < 0.0f ||
            step->action_value > (float)UINT32_MAX / 1000.0f) {
            return ESP_ERR_INVALID_ARG;
        }
        controller->step_deadline_ms = now_ms +
            (uint32_t)(step->action_value * 1000.0f);
        controller->running = true;
        printf(
            "# Calibration delay started: %.1f seconds\n",
            step->action_value);
        return ESP_OK;
    }

    if (step_is_movement(step->type)) {
        const float action_value =
            step->type == ROBOT_STEP_SCAN_ROTATION
                ? controller->scan_rotation_degrees
                : step->action_value;
        error = movement_action_controller_init_with_speed(
            &controller->movement_action_controller,
            step->action.movement,
            action_value,
            step->action_speed_mps);
        if (error == ESP_OK &&
            step->action.movement ==
                MOVEMENT_ACTION_GO_MX_UNTIL_LOCATOR &&
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

    const esp_err_t error =
        start_robot_step(controller, controller->current_step, now_ms);
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
        controller, controller->current_step, now_ms);
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
    if (step_is_delay(step->type)) {
        if (deadline_reached(now_ms, controller->step_deadline_ms)) {
            advance_sequence(controller, now_ms);
        }
        return ESP_OK;
    }
    if (step_is_movement(step->type)) {
        controller->updating_movement = true;
        const bool succeeded = movement_action_controller_update(
            &controller->movement_action_controller);
        controller->updating_movement = false;

        if (succeeded) {
            advance_sequence(controller, now_ms);
        } else if (controller->running) {
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
