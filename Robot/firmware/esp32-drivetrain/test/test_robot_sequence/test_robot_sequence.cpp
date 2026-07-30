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
TapeFollowMode last_follow_mode = TapeFollowMode::SINGLE_SENSOR;
TapeMarkerSensor last_marker_sensor = TapeMarkerSensor::AUTO;
bool service_during_follow = false;
uint32_t fake_millis = 0;

}  // namespace

bool follow_tape(LineFollowerContext *context, Direction direction, float speed_mps,
                 StopCondition stop_condition, float stop_value,
                 float timeout_s, TapeFollowMode mode,
                 TapeMarkerSensor marker_sensor) {
    ++follow_tape_calls;
    last_follow_direction = direction;
    last_follow_speed_mps = speed_mps;
    last_stop_condition = stop_condition;
    last_stop_value = stop_value;
    last_follow_timeout_s = timeout_s;
    last_follow_mode = mode;
    last_marker_sensor = marker_sensor;
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
CommandOpcode sent_commands[16] = {};
size_t sent_command_count = 0;
size_t odometry_ingest_count = 0;
size_t pose_update_count = 0;

struct ControllerFixture {
    UartLink arm_uart = {};
    Pmw3610OdometryLink odometry_link = {};
    PoseTracker pose_tracker = {};
    Drivetrain drivetrain = {};
    RobotSequenceController controller = {};
};

esp_err_t initialize(ControllerFixture *fixture) {
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
    if (packet == nullptr || sent_command_count >= 16) {
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
        case CMD_HABITAT_Z:
            return STATUS_DETAIL_HABITAT_Z_MOVED;
        case CMD_HABITAT_X:
            return STATUS_DETAIL_HABITAT_X_MOVED;
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
    last_follow_mode = TapeFollowMode::SINGLE_SENSOR;
    last_marker_sensor = TapeMarkerSensor::AUTO;
    service_during_follow = false;
    movement_action_controller_set_line_follower_context(nullptr);
}

void tearDown() {}

void test_sequence_waits_for_arm_then_starts_first_action() {
    ControllerFixture fixture = {};
    RobotSequenceController *controller = &fixture.controller;

    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));
    TEST_ASSERT_TRUE(controller->running);
    TEST_ASSERT_TRUE(controller->waiting_for_arm_ready);
    TEST_ASSERT_EQUAL_UINT32(0, sent_command_count);

    robot_sequence_controller_update(controller, 100);
    TEST_ASSERT_EQUAL_UINT32(0, sent_command_count);

    queue_status(STATUS_ACTION_COMPLETE, STATUS_DETAIL_NONE);
    deliver_frame(controller, 101);
    TEST_ASSERT_EQUAL_UINT32(0, controller->current_step);
    TEST_ASSERT_FALSE(controller->waiting_for_arm_ready);
    TEST_ASSERT_EQUAL_UINT32(1, sent_command_count);
    TEST_ASSERT_EQUAL(CMD_TOWER_RETRACT_LOCATOR, sent_commands[0]);

    queue_status(
        STATUS_ACTION_COMPLETE,
        STATUS_DETAIL_TOWER_LOCATOR_RETRACTED);
    deliver_frame(controller, 102);
    TEST_ASSERT_EQUAL_UINT32(1, controller->current_step);
    TEST_ASSERT_EQUAL_UINT32(2, sent_command_count);
    TEST_ASSERT_EQUAL(CMD_TOWER_OPEN_ALL_CLAWS, sent_commands[1]);
}

void test_non_odometry_frames_do_not_advance_movement_steps() {
    ControllerFixture fixture = {};
    RobotSequenceController *controller = &fixture.controller;
    TEST_ASSERT_EQUAL(ESP_OK, initialize(&fixture));

    queue_status(STATUS_ACTION_COMPLETE, STATUS_DETAIL_NONE);
    deliver_frame(controller, 100);
    TEST_ASSERT_EQUAL_UINT32(0, controller->current_step);

    queue_status(STATUS_ACTION_COMPLETE, STATUS_DETAIL_TOWER_HOME);
    deliver_frame(controller, 101);
    TEST_ASSERT_EQUAL_UINT32(0, controller->current_step);

    queue_pi_report(1, PI_RESULT_OK, 0.5f);
    deliver_frame(controller, 102);
    TEST_ASSERT_EQUAL_UINT32(0, controller->current_step);

    queue_status(STATUS_LOCATOR_CONTACT, STATUS_DETAIL_NONE);
    deliver_frame(controller, 103);
    TEST_ASSERT_EQUAL_UINT32(0, controller->current_step);
    TEST_ASSERT_TRUE(controller->locator_contact_pending);
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
            MOVEMENT_ACTION_PX_TAPE_FOLLOW_DISTANCE,
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

    TEST_ASSERT_EQUAL(
        ESP_ERR_INVALID_ARG,
        movement_action_controller_init(
            &controller,
            MOVEMENT_ACTION_PX_TAPE_FOLLOW_DISTANCE,
            -0.1f));
    TEST_ASSERT_EQUAL(
        ESP_ERR_INVALID_ARG,
        movement_action_controller_init(
            &controller, MOVEMENT_ACTION_MAX, 1.0f));
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
            MOVEMENT_ACTION_GO_MX_UNTIL_LOCATOR,
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
        MOVEMENT_ACTION_PX_TAPE_FOLLOW_DISTANCE,
        MOVEMENT_ACTION_MX_TAPE_FOLLOW_DISTANCE,
        MOVEMENT_ACTION_PY_TAPE_FOLLOW_DISTANCE,
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
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.35f, last_follow_speed_mps);
        TEST_ASSERT_EQUAL(
            static_cast<int>(StopCondition::DISTANCE),
            static_cast<int>(last_stop_condition));
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.75f, last_stop_value);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 30.0f, last_follow_timeout_s);
        const TapeFollowMode expected_mode =
            directions[index] == Direction::PY
                ? TapeFollowMode::SINGLE_SENSOR
                : TapeFollowMode::FRONT_BACK_ALIGNED;
        TEST_ASSERT_EQUAL(
            static_cast<int>(expected_mode),
            static_cast<int>(last_follow_mode));
    }
}

void test_habitat_actions_have_unique_completion_details() {
    const CommandOpcode actions[] = {
        CMD_HABITAT_HOME,
        CMD_HABITAT_Z,
        CMD_HABITAT_X,
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

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_sequence_waits_for_arm_then_starts_first_action);
    RUN_TEST(test_non_odometry_frames_do_not_advance_movement_steps);
    RUN_TEST(test_arm_fault_stops_sequence);
    RUN_TEST(test_update_drains_all_uart_packets_and_updates_pose_once);
    RUN_TEST(test_blocking_movement_services_inputs_without_recursive_step_update);
    RUN_TEST(test_movement_action_rejects_invalid_values);
    RUN_TEST(test_locator_contact_notifies_only_locator_approach);
    RUN_TEST(test_tape_distance_actions_route_to_matching_sensor_direction);
    RUN_TEST(test_habitat_actions_have_unique_completion_details);
    return UNITY_END();
}
