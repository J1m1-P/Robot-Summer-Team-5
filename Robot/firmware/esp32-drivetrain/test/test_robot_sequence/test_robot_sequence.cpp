#include <string.h>

#include <unity.h>

extern "C" {
#include "control/task/robot_sequence_controller.h"
#include <robot_common/command_packet.h>
#include <robot_common/odometry_packet.h>
#include <robot_common/pi_action_packet.h>
#include <robot_common/status_packet.h>
}

#include "control/line_following/line_follower.hpp"

namespace {

size_t follow_tape_calls = 0;
Direction last_follow_direction = Direction::PX;
float last_follow_speed_mps = 0.0f;
StopCondition last_stop_condition = StopCondition::TIME_ONLY;
float last_stop_value = 0.0f;
float last_follow_timeout_s = 0.0f;
bool service_during_follow = false;
uint32_t fake_millis = 0;

}  // namespace

bool follow_tape(LineFollowerContext *context, Direction direction, float speed_mps,
                 StopCondition stop_condition, float stop_value,
                 float timeout_s) {
    ++follow_tape_calls;
    last_follow_direction = direction;
    last_follow_speed_mps = speed_mps;
    last_stop_condition = stop_condition;
    last_stop_value = stop_value;
    last_follow_timeout_s = timeout_s;
    if (service_during_follow) {
        TEST_ASSERT_EQUAL(
            ESP_OK,
            robot_sequence_controller_update(
                context->sequence_controller, fake_millis));
    }
    return true;
}

namespace {

PacketFrame queued_packets[16] = {};
size_t queued_packet_count = 0;
size_t queued_packet_head = 0;
CommandOpcode sent_commands[32] = {};
size_t sent_command_count = 0;
size_t odometry_ingest_count = 0;
size_t pose_update_count = 0;

struct ControllerFixture {
    UartLink arm_uart = {};
    Pmw3610OdometryLink odometry_link = {};
    PoseTracker pose_tracker = {};
    Drivetrain drivetrain = {};
    RobotSequenceController controller = {};
    LineFollowerContext line_follower_context = {};
};

esp_err_t initialize(ControllerFixture *fixture) {
    fixture->line_follower_context.sequence_controller = &fixture->controller;
    movement_action_controller_set_line_follower_context(
        &fixture->line_follower_context);
    return robot_sequence_controller_init(
        &fixture->controller,
        &fixture->pose_tracker,
        &fixture->drivetrain,
        &fixture->arm_uart,
        &fixture->odometry_link);
}

void queue_status(StatusCode code, uint8_t detail) {
    TEST_ASSERT_TRUE(queued_packet_count < 16);
    PacketFrame *frame = &queued_packets[queued_packet_count++];
    *frame = {};
    frame->message_type = PACKET_TYPE_STATUS;
    frame->payload_len = STATUS_PACKET_PAYLOAD_SIZE;
    frame->payload[0] = static_cast<uint8_t>(code);
    frame->payload[1] = detail;
}

void queue_pi_report(
    uint8_t request_id,
    PiResultCode result,
    float horizontal_error = 0.0f) {
    const int16_t raw_error =
        static_cast<int16_t>(horizontal_error * 1000.0f);
    TEST_ASSERT_TRUE(queued_packet_count < 16);
    PacketFrame *frame = &queued_packets[queued_packet_count++];
    *frame = {};
    frame->message_type = PACKET_TYPE_PI_REPORT;
    frame->payload_len = PI_REPORT_PACKET_PAYLOAD_SIZE;
    frame->payload[0] = request_id;
    frame->payload[1] = PI_ACTION_SCAN_TELETUBBIES;
    frame->payload[2] = result;
    frame->payload[3] = 1;
    frame->payload[4] = static_cast<uint8_t>(raw_error & 0xFF);
    frame->payload[5] =
        static_cast<uint8_t>((static_cast<uint16_t>(raw_error) >> 8) & 0xFF);
    frame->payload[6] = 90;
}

void deliver_frame(RobotSequenceController *controller, uint32_t now_ms) {
    TEST_ASSERT_EQUAL(
        ESP_OK, robot_sequence_controller_update(controller, now_ms));
}

// Bootstraps the controller, then fast-forwards through the four Tower
// "safe idle position" ARM steps (0-3) and the first checkpoint's tape-follow
// movement (step 4, completed by the fake line follower), landing at step 5
// -- checkpoint 1's ROBOT_STEP_PI_ALIGN,
// with its first CMD_PI_SCAN_TELETUBBIES already sent.
void start_sequence_at_first_checkpoint(
    RobotSequenceController *controller, uint32_t *now_ms) {
    queue_status(STATUS_ACTION_COMPLETE, STATUS_DETAIL_NONE);
    deliver_frame(controller, (*now_ms)++);  // arm ready -> starts step 0

    const uint8_t preamble_details[] = {
        STATUS_DETAIL_TOWER_LOCATOR_RETRACTED,  // step 0: CMD_TOWER_RETRACT_LOCATOR
        STATUS_DETAIL_TOWER_ALL_CLAWS_OPEN,     // step 1: CMD_TOWER_OPEN_ALL_CLAWS
        STATUS_DETAIL_TOWER_Z_MOVED,            // step 2: CMD_TOWER_Z
        STATUS_DETAIL_TOWER_HORIZONTAL,         // step 3: CMD_TOWER_ROTATE_HORIZONTAL
    };
    for (uint8_t detail : preamble_details) {
        queue_status(STATUS_ACTION_COMPLETE, detail);
        deliver_frame(controller, (*now_ms)++);
    }

    // Step 4 completes through the fake line follower, then step 5 sends the
    // first PI scan.
    robot_sequence_controller_update(controller, (*now_ms)++);
}

}  // namespace

extern "C" uint32_t millis(void) {
    return fake_millis;
}

extern "C" const char *esp_err_to_name(esp_err_t) {
    return "test-error";
}

extern "C" esp_err_t uart_link_update(UartLink *) { return ESP_OK; }

extern "C" bool uart_link_has_packet(const UartLink *) {
    return queued_packet_head < queued_packet_count;
}

extern "C" esp_err_t uart_link_take_packet(
    UartLink *,
    PacketFrame *packet_out) {
    if (packet_out == nullptr || queued_packet_head >= queued_packet_count) {
        return ESP_ERR_NOT_FOUND;
    }
    *packet_out = queued_packets[queued_packet_head++];
    return ESP_OK;
}

extern "C" esp_err_t command_packet_send(
    UartLink *,
    const CommandPacket *packet) {
    if (packet == nullptr || sent_command_count >= 32) {
        return ESP_ERR_INVALID_ARG;
    }
    sent_commands[sent_command_count++] = packet->opcode;
    return ESP_OK;
}

extern "C" bool odometry_packet_is(const PacketFrame *frame) {
    return frame != nullptr && frame->message_type == PACKET_TYPE_ODOMETRY;
}

extern "C" void odometry_link_ingest(
    Pmw3610OdometryLink *link,
    const PacketFrame *) {
    ++odometry_ingest_count;
    link->has_packet = true;
}

extern "C" DrivetrainWheelCounts drivetrain_get_wheel_counts(
    const Drivetrain *) {
    return {};
}

extern "C" esp_err_t pose_tracker_update(
    PoseTracker *,
    const DrivetrainWheelCounts *,
    const OdometryPacket *) {
    ++pose_update_count;
    return ESP_OK;
}

extern "C" bool status_packet_is(const PacketFrame *frame) {
    return frame != nullptr &&
           frame->message_type == PACKET_TYPE_STATUS &&
           frame->payload_len == STATUS_PACKET_PAYLOAD_SIZE;
}

extern "C" esp_err_t status_packet_decode(
    const PacketFrame *frame,
    StatusPacket *packet_out) {
    if (!status_packet_is(frame) || packet_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    packet_out->code = static_cast<StatusCode>(frame->payload[0]);
    packet_out->detail = frame->payload[1];
    return ESP_OK;
}

extern "C" bool pi_report_packet_is(const PacketFrame *frame) {
    return frame != nullptr &&
           frame->message_type == PACKET_TYPE_PI_REPORT &&
           frame->payload_len == PI_REPORT_PACKET_PAYLOAD_SIZE;
}

extern "C" esp_err_t pi_report_packet_decode(
    const PacketFrame *frame,
    PiReportPacket *packet_out) {
    if (!pi_report_packet_is(frame) || packet_out == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    const int16_t raw_error = static_cast<int16_t>(
        static_cast<uint16_t>(frame->payload[4]) |
        (static_cast<uint16_t>(frame->payload[5]) << 8));
    packet_out->request_id = frame->payload[0];
    packet_out->action = static_cast<PiAction>(frame->payload[1]);
    packet_out->result = static_cast<PiResultCode>(frame->payload[2]);
    packet_out->target_id = frame->payload[3];
    packet_out->horizontal_error = raw_error / 1000.0f;
    packet_out->confidence_percent = frame->payload[6];
    return ESP_OK;
}

// Test-local mock -- kept self-consistent with the CommandOpcode/
// ActionStatusDetail enums, not necessarily identical to the real arm-side
// mapping in status_packet.c.
extern "C" ActionStatusDetail arm_action_status_detail(
    CommandOpcode command) {
    switch (command) {
        case CMD_TOWER_HOME:
            return STATUS_DETAIL_TOWER_HOME;
        case CMD_TOWER_Z:
            return STATUS_DETAIL_TOWER_Z_MOVED;
        case CMD_TOWER_X:
            return STATUS_DETAIL_TOWER_X_MOVED;
        case CMD_TOWER_ROTATE_VERTICAL:
            return STATUS_DETAIL_TOWER_VERTICAL;
        case CMD_TOWER_ROTATE_HORIZONTAL:
            return STATUS_DETAIL_TOWER_HORIZONTAL;
        case CMD_TOWER_OPEN_ALL_CLAWS:
            return STATUS_DETAIL_TOWER_ALL_CLAWS_OPEN;
        case CMD_TOWER_CLOSE_ALL_CLAWS:
            return STATUS_DETAIL_TOWER_ALL_CLAWS_CLOSED;
        case CMD_TOWER_OPEN_LEFT_CLAW:
            return STATUS_DETAIL_TOWER_LEFT_CLAW_OPEN;
        case CMD_TOWER_CLOSE_LEFT_CLAW:
            return STATUS_DETAIL_TOWER_LEFT_CLAW_CLOSED;
        case CMD_TOWER_OPEN_MIDDLE_CLAW:
            return STATUS_DETAIL_TOWER_MIDDLE_CLAW_OPEN;
        case CMD_TOWER_CLOSE_MIDDLE_CLAW:
            return STATUS_DETAIL_TOWER_MIDDLE_CLAW_CLOSED;
        case CMD_TOWER_OPEN_RIGHT_CLAW:
            return STATUS_DETAIL_TOWER_RIGHT_CLAW_OPEN;
        case CMD_TOWER_CLOSE_RIGHT_CLAW:
            return STATUS_DETAIL_TOWER_RIGHT_CLAW_CLOSED;
        case CMD_TOWER_EXTEND_LOCATOR:
            return STATUS_DETAIL_TOWER_LOCATOR_EXTENDED;
        case CMD_TOWER_RETRACT_LOCATOR:
            return STATUS_DETAIL_TOWER_LOCATOR_RETRACTED;
        case CMD_HABITAT_HOME:
            return STATUS_DETAIL_HABITAT_HOME;
        case CMD_HABITAT_Z_UP:
            return STATUS_DETAIL_HABITAT_Z_RAISED;
        case CMD_HABITAT_Z_DOWN:
            return STATUS_DETAIL_HABITAT_Z_LOWERED;
        case CMD_HABITAT_X_LEFT:
            return STATUS_DETAIL_HABITAT_X_LEFT;
        case CMD_HABITAT_X_RIGHT:
            return STATUS_DETAIL_HABITAT_X_RIGHT;
        case CMD_HABITAT_OPEN_CLAWS:
            return STATUS_DETAIL_HABITAT_CLAWS_OPEN;
        case CMD_HABITAT_CLOSE_CLAWS:
            return STATUS_DETAIL_HABITAT_CLAWS_CLOSED;
        case CMD_HABITAT_OPEN_LEFT_CLAW:
            return STATUS_DETAIL_HABITAT_LEFT_CLAW_OPEN;
        case CMD_HABITAT_CLOSE_LEFT_CLAW:
            return STATUS_DETAIL_HABITAT_LEFT_CLAW_CLOSED;
        case CMD_HABITAT_OPEN_RIGHT_CLAW:
            return STATUS_DETAIL_HABITAT_RIGHT_CLAW_OPEN;
        case CMD_HABITAT_CLOSE_RIGHT_CLAW:
            return STATUS_DETAIL_HABITAT_RIGHT_CLAW_CLOSED;
        default:
            return STATUS_DETAIL_NONE;
    }
}

void setUp() {
    fake_millis = 100;
    memset(queued_packets, 0, sizeof(queued_packets));
    queued_packet_count = 0;
    queued_packet_head = 0;
    memset(sent_commands, 0, sizeof(sent_commands));
    sent_command_count = 0;
    odometry_ingest_count = 0;
    pose_update_count = 0;
    follow_tape_calls = 0;
    last_follow_direction = Direction::PX;
    last_follow_speed_mps = 0.0f;
    last_stop_condition = StopCondition::TIME_ONLY;
    last_stop_value = 0.0f;
    last_follow_timeout_s = 0.0f;
    service_during_follow = false;
    movement_action_controller_set_line_follower_context(nullptr);
}

void tearDown() {}

void test_sequence_waits_for_arm_then_starts_first_checkpoint() {
    ControllerFixture fixture = {};
    RobotSequenceController *controller = &fixture.controller;

    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));
    TEST_ASSERT_TRUE(controller->running);
    TEST_ASSERT_TRUE(controller->waiting_for_arm_ready);
    TEST_ASSERT_EQUAL_UINT32(0, sent_command_count);

    uint32_t now_ms = 100;
    start_sequence_at_first_checkpoint(controller, &now_ms);

    // Steps 0-3 (retract locator, open all claws, rotate horizontal, Z) each
    // sent one ARM command; step 4's tape-follow doesn't send a command; step
    // 5 (first PI_ALIGN) sent the first PI_SCAN.
    TEST_ASSERT_EQUAL_UINT32(5, controller->current_step);
    TEST_ASSERT_EQUAL_UINT32(5, sent_command_count);
    TEST_ASSERT_EQUAL(CMD_TOWER_RETRACT_LOCATOR, sent_commands[0]);
    TEST_ASSERT_EQUAL(CMD_TOWER_OPEN_ALL_CLAWS, sent_commands[1]);
    TEST_ASSERT_EQUAL(CMD_TOWER_Z, sent_commands[2]);
    TEST_ASSERT_EQUAL(CMD_TOWER_ROTATE_HORIZONTAL, sent_commands[3]);
    TEST_ASSERT_EQUAL(CMD_PI_SCAN_TELETUBBIES, sent_commands[4]);
}

void test_checkpoint_not_found_advances_to_next_checkpoint() {
    ControllerFixture fixture = {};
    RobotSequenceController *controller = &fixture.controller;
    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));

    uint32_t now_ms = 100;
    start_sequence_at_first_checkpoint(controller, &now_ms);
    TEST_ASSERT_EQUAL_UINT32(5, controller->current_step);

    // A miss completes the PI_ALIGN step immediately (nothing to recover
    // from -- no rotation has happened yet this attempt).
    queue_pi_report(1, PI_RESULT_NOT_FOUND);
    deliver_frame(controller, now_ms++);
    // This update also completes checkpoint 2's fake tape movement and starts
    // its scan.
    TEST_ASSERT_EQUAL_UINT32(7, controller->current_step);
    TEST_ASSERT_EQUAL(CMD_PI_SCAN_TELETUBBIES, sent_commands[sent_command_count - 1]);
}

void test_uncentered_scan_rotates_and_rescans_in_one_call() {
    ControllerFixture fixture = {};
    RobotSequenceController *controller = &fixture.controller;
    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));

    uint32_t now_ms = 100;
    start_sequence_at_first_checkpoint(controller, &now_ms);
    const size_t sent_before = sent_command_count;

    // Rotation is now a single blocking call inside service_pi_align, so one
    // deliver_frame() does the whole rotate-then-rescan -- no separate tick
    // needed to let the rotation "complete".
    queue_pi_report(1, PI_RESULT_OK, 0.5f);
    deliver_frame(controller, now_ms++);

    TEST_ASSERT_EQUAL_UINT32(5, controller->current_step);  // still the same checkpoint
    TEST_ASSERT_EQUAL_UINT32(1, controller->align_attempts);
    TEST_ASSERT_EQUAL_UINT32(sent_before + 1, sent_command_count);  // the re-scan
    TEST_ASSERT_EQUAL(CMD_PI_SCAN_TELETUBBIES, sent_commands[sent_command_count - 1]);
}

void test_centered_scan_advances_without_rotating() {
    ControllerFixture fixture = {};
    RobotSequenceController *controller = &fixture.controller;
    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));

    uint32_t now_ms = 100;
    start_sequence_at_first_checkpoint(controller, &now_ms);
    const size_t sent_before = sent_command_count;

    // A small error (already centered, so the Pi would have flashed) should
    // complete the step immediately instead of rotating.
    queue_pi_report(1, PI_RESULT_OK, 0.01f);
    deliver_frame(controller, now_ms++);

    TEST_ASSERT_EQUAL_UINT32(7, controller->current_step);  // checkpoint 2's PI_ALIGN
    // No alignment re-scan; the one new command starts checkpoint 2's scan.
    TEST_ASSERT_EQUAL_UINT32(sent_before + 1, sent_command_count);
}

void test_reposition_does_not_count_as_attempt() {
    ControllerFixture fixture = {};
    RobotSequenceController *controller = &fixture.controller;
    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));

    uint32_t now_ms = 100;
    start_sequence_at_first_checkpoint(controller, &now_ms);
    TEST_ASSERT_EQUAL_UINT32(5, controller->current_step);

    // More repositioning rounds than ALIGN_MAX_ATTEMPTS should all rotate and
    // re-scan -- one deliver_frame() each, since the rotation is now a single
    // blocking call -- without ever touching align_attempts. The 0.3f error
    // value is ignored by the ESP (REPOSITION undoes its own tracked net
    // rotation, which is 0 here since no real detection ever rotated) -- this
    // test only cares about the attempt-budget bookkeeping, not the
    // magnitude.
    for (uint8_t round = 0; round < 6; ++round) {
        queue_pi_report(round + 1, PI_RESULT_REPOSITION, 0.3f);
        deliver_frame(controller, now_ms++);
        TEST_ASSERT_EQUAL_UINT32(5, controller->current_step);  // still the same checkpoint
        TEST_ASSERT_EQUAL_UINT32(0, controller->align_attempts);
    }

    // A real (large-error) OK report afterward should still get the full
    // attempt budget -- confirms the reposition rounds didn't consume it.
    queue_pi_report(100, PI_RESULT_OK, 0.5f);
    deliver_frame(controller, now_ms++);
    TEST_ASSERT_EQUAL_UINT32(1, controller->align_attempts);
}

void test_align_gives_up_after_max_attempts() {
    ControllerFixture fixture = {};
    RobotSequenceController *controller = &fixture.controller;
    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));

    uint32_t now_ms = 100;
    start_sequence_at_first_checkpoint(controller, &now_ms);

    // Report a large, never-improving error every round. After
    // ALIGN_MAX_ATTEMPTS rotations the step should give up and advance
    // rather than retry forever.
    uint8_t request_id = 1;
    for (uint8_t attempt = 0; attempt < 4; ++attempt) {
        queue_pi_report(request_id++, PI_RESULT_OK, 0.5f);
        deliver_frame(controller, now_ms++);
        if (controller->current_step != 5) break;
    }

    TEST_ASSERT_EQUAL_UINT32(7, controller->current_step);  // checkpoint 2's PI_ALIGN
}

void test_non_matching_frames_do_not_advance_current_step() {
    ControllerFixture fixture = {};
    RobotSequenceController *controller = &fixture.controller;
    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));

    queue_status(STATUS_ACTION_COMPLETE, STATUS_DETAIL_NONE);
    deliver_frame(controller, 100);
    TEST_ASSERT_EQUAL_UINT32(0, controller->current_step);

    queue_status(STATUS_ACTION_COMPLETE, STATUS_DETAIL_TOWER_HOME);
    deliver_frame(controller, 101);
    TEST_ASSERT_EQUAL_UINT32(0, controller->current_step);

    // Step 0 isn't a PI_ALIGN step, so a stray Pi report is ignored too.
    queue_pi_report(1, PI_RESULT_OK, 0.5f);
    deliver_frame(controller, 102);
    TEST_ASSERT_EQUAL_UINT32(0, controller->current_step);

    queue_status(STATUS_LOCATOR_CONTACT, STATUS_DETAIL_NONE);
    deliver_frame(controller, 103);
    TEST_ASSERT_EQUAL_UINT32(0, controller->current_step);
    TEST_ASSERT_TRUE(controller->locator_contact_pending);
}

void test_not_found_after_rotation_retries_then_gives_up_checkpoint_only() {
    ControllerFixture fixture = {};
    RobotSequenceController *controller = &fixture.controller;
    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));

    uint32_t now_ms = 100;
    start_sequence_at_first_checkpoint(controller, &now_ms);

    // A real detection that isn't centered establishes last_rotation_degrees
    // != 0, so a later NOT_FOUND has something to recover from instead of
    // being treated as "nothing here" (see
    // test_checkpoint_not_found_advances_to_next_checkpoint for that case).
    queue_pi_report(1, PI_RESULT_OK, 0.5f);
    deliver_frame(controller, now_ms++);
    TEST_ASSERT_EQUAL_UINT32(5, controller->current_step);
    TEST_ASSERT_EQUAL_UINT32(1, controller->align_attempts);

    // The target is now lost (e.g. that rotation overshot). NOT_FOUND should
    // retry with a recovery rotation instead of giving up immediately, until
    // attempts run out.
    uint8_t request_id = 2;
    for (uint8_t attempt = 0; attempt < 4; ++attempt) {
        queue_pi_report(request_id++, PI_RESULT_NOT_FOUND);
        deliver_frame(controller, now_ms++);
        if (controller->current_step != 5) break;
        TEST_ASSERT_TRUE(controller->running);
    }

    // Gave up on just this checkpoint once attempts ran out -- the sequence
    // itself keeps running (advances to the next checkpoint).
    TEST_ASSERT_TRUE(controller->running);
    TEST_ASSERT_EQUAL_UINT32(7, controller->current_step);
}

void test_camera_fault_retries_instead_of_aborting_sequence() {
    ControllerFixture fixture = {};
    RobotSequenceController *controller = &fixture.controller;
    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));

    uint32_t now_ms = 100;
    start_sequence_at_first_checkpoint(controller, &now_ms);

    // Unlike NOT_FOUND, a camera fault isn't about whether we've rotated yet
    // -- it should always retry, even on the very first scan of a checkpoint.
    queue_pi_report(1, PI_RESULT_CAMERA_FAULT);
    deliver_frame(controller, now_ms++);
    TEST_ASSERT_TRUE(controller->running);
    TEST_ASSERT_EQUAL_UINT32(5, controller->current_step);  // still the same checkpoint
    TEST_ASSERT_EQUAL_UINT32(1, controller->align_attempts);
}

void test_all_found_skips_remaining_scans_but_finishes_route() {
    ControllerFixture fixture = {};
    RobotSequenceController *controller = &fixture.controller;
    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));

    uint32_t now_ms = 100;
    start_sequence_at_first_checkpoint(controller, &now_ms);
    const size_t sent_before = sent_command_count;

    // Finding both targets stops later scans, but the robot must still drive
    // the rest of line's 4.8 m search section before turning toward the tower.
    queue_pi_report(1, PI_RESULT_ALL_FOUND);
    deliver_frame(controller, now_ms++);

    // The first remaining travel step completes in the report's update.
    TEST_ASSERT_EQUAL_UINT32(8, controller->current_step);
    robot_sequence_controller_update(controller, now_ms++);  // checkpoint 3 travel
    TEST_ASSERT_EQUAL_UINT32(10, controller->current_step);
    robot_sequence_controller_update(controller, now_ms++);  // final 1.2 m travel
    TEST_ASSERT_EQUAL_UINT32(11, controller->current_step);  // rotate-to-tower step
    TEST_ASSERT_EQUAL_UINT32(sent_before, sent_command_count);  // no more PI_SCANs sent
    TEST_ASSERT_TRUE(controller->running);
}

void test_all_found_repositions_before_skipping_when_rotation_needed() {
    ControllerFixture fixture = {};
    RobotSequenceController *controller = &fixture.controller;
    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));

    uint32_t now_ms = 100;
    start_sequence_at_first_checkpoint(controller, &now_ms);

    // A real detection that rotates first establishes chase_net_rotation_degrees
    // != 0. ALL_FOUND ignores whatever horizontal_error the Pi sends -- the
    // ESP undoes ITS OWN tracked net rotation, not a Pi-reported magnitude
    // (see chase_net_rotation_degrees in robot_sequence_controller.c).
    queue_pi_report(1, PI_RESULT_OK, 0.4f);
    deliver_frame(controller, now_ms++);
    TEST_ASSERT_EQUAL_UINT32(5, controller->current_step);  // still checkpoint 1

    // The undo rotation happens immediately; later scans are skipped while
    // the remaining route movements continue normally.
    queue_pi_report(2, PI_RESULT_ALL_FOUND, 0.0f);
    deliver_frame(controller, now_ms++);

    TEST_ASSERT_EQUAL_UINT32(8, controller->current_step);
    TEST_ASSERT_TRUE(controller->running);
}

void test_arm_fault_stops_sequence() {
    ControllerFixture fixture = {};
    RobotSequenceController *controller = &fixture.controller;
    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));

    queue_status(STATUS_ACTION_COMPLETE, STATUS_DETAIL_NONE);
    deliver_frame(controller, 100);
    robot_sequence_controller_update(controller, 101);
    queue_status(STATUS_FAULT, STATUS_DETAIL_TOWER_HOME);
    deliver_frame(controller, 102);

    TEST_ASSERT_FALSE(controller->running);
}

void test_update_drains_all_uart_packets_and_updates_pose_once() {
    ControllerFixture fixture = {};
    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));

    queued_packets[queued_packet_count++].message_type = PACKET_TYPE_ODOMETRY;
    queued_packets[queued_packet_count++].message_type = 0xFE;
    queued_packets[queued_packet_count++].message_type = PACKET_TYPE_ODOMETRY;

    TEST_ASSERT_EQUAL(
        ESP_OK,
        robot_sequence_controller_update(&fixture.controller, 100));
    TEST_ASSERT_EQUAL_UINT32(3, queued_packet_head);
    TEST_ASSERT_EQUAL_UINT32(2, odometry_ingest_count);
    TEST_ASSERT_EQUAL_UINT32(1, pose_update_count);
}

void test_blocking_movement_services_inputs_without_recursive_step_update() {
    ControllerFixture fixture = {};
    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));

    fixture.controller.waiting_for_arm_ready = false;
    fixture.controller.current_step = 4;  // first tape-follow step
    TEST_ASSERT_EQUAL(
        ESP_OK,
        movement_action_controller_init(
            &fixture.controller.movement_action_controller,
            MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE,
            4.8f));

    LineFollowerContext context = {};
    context.sequence_controller = &fixture.controller;
    movement_action_controller_set_line_follower_context(&context);
    service_during_follow = true;

    TEST_ASSERT_EQUAL(
        ESP_OK,
        robot_sequence_controller_update(&fixture.controller, 100));
    TEST_ASSERT_EQUAL_UINT32(1, follow_tape_calls);
    TEST_ASSERT_EQUAL_UINT32(2, pose_update_count);
    TEST_ASSERT_EQUAL_UINT32(5, fixture.controller.current_step);
}

void test_movement_action_rejects_invalid_values() {
    MovementActionController controller = {};

    // Only the tape-follow-distance actions require a non-negative value.
    TEST_ASSERT_EQUAL(
        ESP_ERR_INVALID_ARG,
        movement_action_controller_init(
            &controller,
            MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE,
            -0.1f));
    TEST_ASSERT_EQUAL(
        ESP_ERR_INVALID_ARG,
        movement_action_controller_init(
            &controller, MOVEMENT_ACTION_MAX, 1.0f));
    // GO_X_DISTANCE and ROTATE allow negative values (direction/sign-bearing).
    TEST_ASSERT_EQUAL(
        ESP_OK,
        movement_action_controller_init(
            &controller, MOVEMENT_ACTION_GO_X_DISTANCE, -0.05f));
    TEST_ASSERT_EQUAL(
        ESP_OK,
        movement_action_controller_init(
            &controller, MOVEMENT_ACTION_ROTATE, -90.0f));
}

void test_locator_contact_notifies_only_locator_approach() {
    MovementActionController controller = {};
    TEST_ASSERT_EQUAL(
        ESP_OK,
        movement_action_controller_init(
            &controller,
            MOVEMENT_ACTION_GO_BACKWARD_UNTIL_LOCATOR,
            0.0f));
    TEST_ASSERT_FALSE(controller.locator_contact_detected);

    movement_action_controller_notify_locator_contact(&controller);
    TEST_ASSERT_TRUE(controller.locator_contact_detected);

    TEST_ASSERT_EQUAL(
        ESP_OK,
        movement_action_controller_init(
            &controller,
            MOVEMENT_ACTION_ROTATE,
            90.0f));
    movement_action_controller_notify_locator_contact(&controller);
    TEST_ASSERT_FALSE(controller.locator_contact_detected);
}

void test_tape_distance_actions_route_to_matching_sensor_direction() {
    LineFollowerContext context = {};
    movement_action_controller_set_line_follower_context(&context);

    const MovementAction actions[] = {
        MOVEMENT_ACTION_FRONT_TAPE_FOLLOW_DISTANCE,
        MOVEMENT_ACTION_BACK_TAPE_FOLLOW_DISTANCE,
        MOVEMENT_ACTION_LEFT_TAPE_FOLLOW_DISTANCE,
    };
    const Direction directions[] = {
        Direction::PX,
        Direction::MX,
        Direction::PY,
    };

    for (size_t index = 0; index < 3; ++index) {
        MovementActionController controller = {};
        TEST_ASSERT_EQUAL(
            ESP_OK,
            movement_action_controller_init(
                &controller, actions[index], 0.75f));
        TEST_ASSERT_TRUE(movement_action_controller_update(&controller));
        TEST_ASSERT_EQUAL_UINT32(index + 1, follow_tape_calls);
        TEST_ASSERT_EQUAL(
            static_cast<int>(directions[index]),
            static_cast<int>(last_follow_direction));
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.5f, last_follow_speed_mps);
        TEST_ASSERT_EQUAL(
            static_cast<int>(StopCondition::DISTANCE),
            static_cast<int>(last_stop_condition));
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.75f, last_stop_value);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, last_follow_timeout_s);
    }
}

void test_habitat_actions_have_unique_completion_details() {
    const CommandOpcode actions[] = {
        CMD_HABITAT_HOME,
        CMD_HABITAT_Z_UP,
        CMD_HABITAT_Z_DOWN,
        CMD_HABITAT_X_LEFT,
        CMD_HABITAT_X_RIGHT,
        CMD_HABITAT_OPEN_CLAWS,
        CMD_HABITAT_CLOSE_CLAWS,
        CMD_HABITAT_OPEN_LEFT_CLAW,
        CMD_HABITAT_CLOSE_LEFT_CLAW,
        CMD_HABITAT_OPEN_RIGHT_CLAW,
        CMD_HABITAT_CLOSE_RIGHT_CLAW,
    };

    for (size_t index = 0; index < sizeof(actions) / sizeof(actions[0]);
         ++index) {
        const ActionStatusDetail detail =
            arm_action_status_detail(actions[index]);
        TEST_ASSERT_NOT_EQUAL(STATUS_DETAIL_NONE, detail);
        for (size_t prior = 0; prior < index; ++prior) {
            TEST_ASSERT_NOT_EQUAL(
                arm_action_status_detail(actions[prior]),
                detail);
        }
    }
}

void test_tower_actions_have_unique_completion_details() {
    const CommandOpcode actions[] = {
        CMD_TOWER_HOME,
        CMD_TOWER_Z,
        CMD_TOWER_X,
        CMD_TOWER_ROTATE_VERTICAL,
        CMD_TOWER_ROTATE_HORIZONTAL,
        CMD_TOWER_OPEN_ALL_CLAWS,
        CMD_TOWER_CLOSE_ALL_CLAWS,
        CMD_TOWER_OPEN_LEFT_CLAW,
        CMD_TOWER_CLOSE_LEFT_CLAW,
        CMD_TOWER_OPEN_MIDDLE_CLAW,
        CMD_TOWER_CLOSE_MIDDLE_CLAW,
        CMD_TOWER_OPEN_RIGHT_CLAW,
        CMD_TOWER_CLOSE_RIGHT_CLAW,
        CMD_TOWER_EXTEND_LOCATOR,
        CMD_TOWER_RETRACT_LOCATOR,
    };

    for (size_t index = 0; index < sizeof(actions) / sizeof(actions[0]);
         ++index) {
        const ActionStatusDetail detail =
            arm_action_status_detail(actions[index]);
        TEST_ASSERT_NOT_EQUAL(STATUS_DETAIL_NONE, detail);
        for (size_t prior = 0; prior < index; ++prior) {
            TEST_ASSERT_NOT_EQUAL(
                arm_action_status_detail(actions[prior]),
                detail);
        }
    }
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_sequence_waits_for_arm_then_starts_first_checkpoint);
    RUN_TEST(test_checkpoint_not_found_advances_to_next_checkpoint);
    RUN_TEST(test_uncentered_scan_rotates_and_rescans_in_one_call);
    RUN_TEST(test_centered_scan_advances_without_rotating);
    RUN_TEST(test_reposition_does_not_count_as_attempt);
    RUN_TEST(test_align_gives_up_after_max_attempts);
    RUN_TEST(test_non_matching_frames_do_not_advance_current_step);
    RUN_TEST(test_not_found_after_rotation_retries_then_gives_up_checkpoint_only);
    RUN_TEST(test_camera_fault_retries_instead_of_aborting_sequence);
    RUN_TEST(test_all_found_skips_remaining_scans_but_finishes_route);
    RUN_TEST(test_all_found_repositions_before_skipping_when_rotation_needed);
    RUN_TEST(test_arm_fault_stops_sequence);
    RUN_TEST(test_update_drains_all_uart_packets_and_updates_pose_once);
    RUN_TEST(test_blocking_movement_services_inputs_without_recursive_step_update);
    RUN_TEST(test_movement_action_rejects_invalid_values);
    RUN_TEST(test_locator_contact_notifies_only_locator_approach);
    RUN_TEST(test_tape_distance_actions_route_to_matching_sensor_direction);
    RUN_TEST(test_habitat_actions_have_unique_completion_details);
    RUN_TEST(test_tower_actions_have_unique_completion_details);
    return UNITY_END();
}
