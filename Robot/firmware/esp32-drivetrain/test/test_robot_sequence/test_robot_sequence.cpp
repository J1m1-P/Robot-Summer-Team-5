#include <string.h>

#include <unity.h>

extern "C" {
#include "control/task/robot_sequence_controller.h"
#include <robot_common/command_packet.h>
#include <robot_common/odometry_packet.h>
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

// kRobotSequence is currently empty (the sequence is being redesigned), so
// these only cover the step machinery and data mappings that don't depend on
// its contents. Sequence-flow tests (checkpoint ordering, step transitions,
// etc.) belong back here once the new sequence is written.

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
    RUN_TEST(test_update_drains_all_uart_packets_and_updates_pose_once);
    RUN_TEST(test_movement_action_rejects_invalid_values);
    RUN_TEST(test_locator_contact_notifies_only_locator_approach);
    RUN_TEST(test_tape_distance_actions_route_to_matching_sensor_direction);
    RUN_TEST(test_habitat_actions_have_unique_completion_details);
    RUN_TEST(test_tower_actions_have_unique_completion_details);
    return UNITY_END();
}
