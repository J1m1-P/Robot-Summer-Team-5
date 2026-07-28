#include <string.h>

#include <unity.h>

extern "C" {
#include "control/task/robot_sequence_controller.h"
#include <robot_common/command_packet.h>
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

}  // namespace

bool follow_tape(LineFollowerContext *, Direction direction, float speed_mps,
                 StopCondition stop_condition, float stop_value,
                 float timeout_s) {
    ++follow_tape_calls;
    last_follow_direction = direction;
    last_follow_speed_mps = speed_mps;
    last_stop_condition = stop_condition;
    last_stop_value = stop_value;
    last_follow_timeout_s = timeout_s;
    return true;
}

namespace {

uint32_t fake_millis = 0;
PacketFrame queued_packet = {};
CommandOpcode sent_commands[16] = {};
size_t sent_command_count = 0;

void queue_status(StatusCode code, uint8_t detail) {
    queued_packet = {};
    queued_packet.message_type = PACKET_TYPE_STATUS;
    queued_packet.payload_len = STATUS_PACKET_PAYLOAD_SIZE;
    queued_packet.payload[0] = static_cast<uint8_t>(code);
    queued_packet.payload[1] = detail;
}

void queue_pi_report(
    uint8_t request_id,
    PiResultCode result,
    float horizontal_error = 0.0f) {
    const int16_t raw_error =
        static_cast<int16_t>(horizontal_error * 1000.0f);
    queued_packet = {};
    queued_packet.message_type = PACKET_TYPE_PI_REPORT;
    queued_packet.payload_len = PI_REPORT_PACKET_PAYLOAD_SIZE;
    queued_packet.payload[0] = request_id;
    queued_packet.payload[1] = PI_ACTION_SCAN_TELETUBBIES;
    queued_packet.payload[2] = result;
    queued_packet.payload[3] = 1;
    queued_packet.payload[4] = static_cast<uint8_t>(raw_error & 0xFF);
    queued_packet.payload[5] =
        static_cast<uint8_t>((static_cast<uint16_t>(raw_error) >> 8) & 0xFF);
    queued_packet.payload[6] = 90;
}

// Delivers the most recently queued frame directly, the way pose_service's
// dispatcher would after dequeuing it from arm_uart.
void deliver_frame(RobotSequenceController *controller, uint32_t now_ms) {
    robot_sequence_controller_handle_frame(controller, &queued_packet, now_ms);
}

}  // namespace

extern "C" uint32_t millis(void) {
    return fake_millis;
}

extern "C" const char *esp_err_to_name(esp_err_t) {
    return "test-error";
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
        case CMD_TOWER_Z_UP:
            return STATUS_DETAIL_TOWER_Z_RAISED;
        case CMD_TOWER_Z_DOWN:
            return STATUS_DETAIL_TOWER_Z_LOWERED;
        case CMD_TOWER_X_LEFT:
            return STATUS_DETAIL_TOWER_X_LEFT;
        case CMD_TOWER_X_RIGHT:
            return STATUS_DETAIL_TOWER_X_RIGHT;
        case CMD_TOWER_ROTATE_VERTICAL:
            return STATUS_DETAIL_TOWER_VERTICAL;
        case CMD_TOWER_ROTATE_HORIZONTAL:
            return STATUS_DETAIL_TOWER_HORIZONTAL;
        case CMD_TOWER_OPEN_CLAW:
            return STATUS_DETAIL_TOWER_CLAW_OPEN;
        case CMD_TOWER_CLOSE_CLAW:
            return STATUS_DETAIL_TOWER_CLAW_CLOSED;
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
    queued_packet = {};
    memset(sent_commands, 0, sizeof(sent_commands));
    sent_command_count = 0;
    follow_tape_calls = 0;
    last_follow_direction = Direction::PX;
    last_follow_speed_mps = 0.0f;
    last_stop_condition = StopCondition::TIME_ONLY;
    last_stop_value = 0.0f;
    last_follow_timeout_s = 0.0f;
    movement_action_controller_set_line_follower_context(nullptr);
}

void tearDown() {}

void test_sequence_waits_for_arm_then_runs_square() {
    UartLink arm_uart = {};
    RobotSequenceController controller = {};

    TEST_ASSERT_EQUAL(
        ESP_OK,
        robot_sequence_controller_init(&controller, &arm_uart));
    TEST_ASSERT_TRUE(controller.running);
    TEST_ASSERT_TRUE(controller.waiting_for_arm_ready);
    TEST_ASSERT_EQUAL_UINT32(0, sent_command_count);

    robot_sequence_controller_update(&controller, 100);
    TEST_ASSERT_EQUAL_UINT32(0, sent_command_count);

    queue_status(STATUS_ACTION_COMPLETE, STATUS_DETAIL_NONE);
    deliver_frame(&controller, 101);
    TEST_ASSERT_EQUAL_UINT32(0, controller.current_step);
    TEST_ASSERT_FALSE(controller.waiting_for_arm_ready);
    TEST_ASSERT_EQUAL_UINT32(0, sent_command_count);

    for (size_t step = 0; step < 8; ++step) {
        const MovementActionController *movement =
            &controller.movement_action_controller;
        if (step % 2 == 0) {
            TEST_ASSERT_EQUAL(
                MOVEMENT_ACTION_GO_FORWARD, movement->action);
            TEST_ASSERT_FLOAT_WITHIN(
                0.001f, 0.25f, movement->action_value);
        } else {
            TEST_ASSERT_EQUAL(MOVEMENT_ACTION_ROTATE, movement->action);
            TEST_ASSERT_FLOAT_WITHIN(
                0.001f, 90.0f, movement->action_value);
        }
        robot_sequence_controller_update(
            &controller, static_cast<uint32_t>(102 + step));
    }

    TEST_ASSERT_EQUAL_UINT32(8, controller.current_step);
    TEST_ASSERT_FALSE(controller.running);
    TEST_ASSERT_EQUAL_UINT32(0, sent_command_count);
}

void test_non_odometry_frames_do_not_advance_movement_steps() {
    UartLink arm_uart = {};
    RobotSequenceController controller = {};
    TEST_ASSERT_EQUAL(
        ESP_OK,
        robot_sequence_controller_init(&controller, &arm_uart));

    queue_status(STATUS_ACTION_COMPLETE, STATUS_DETAIL_NONE);
    deliver_frame(&controller, 100);
    TEST_ASSERT_EQUAL_UINT32(0, controller.current_step);

    queue_status(STATUS_ACTION_COMPLETE, STATUS_DETAIL_TOWER_HOME);
    deliver_frame(&controller, 101);
    TEST_ASSERT_EQUAL_UINT32(0, controller.current_step);

    queue_pi_report(1, PI_RESULT_OK, 0.5f);
    deliver_frame(&controller, 102);
    TEST_ASSERT_EQUAL_UINT32(0, controller.current_step);
}

void test_arm_fault_stops_sequence() {
    UartLink arm_uart = {};
    RobotSequenceController controller = {};
    TEST_ASSERT_EQUAL(
        ESP_OK,
        robot_sequence_controller_init(&controller, &arm_uart));

    queue_status(STATUS_ACTION_COMPLETE, STATUS_DETAIL_NONE);
    deliver_frame(&controller, 100);
    robot_sequence_controller_update(&controller, 101);
    queue_status(STATUS_FAULT, STATUS_DETAIL_TOWER_HOME);
    deliver_frame(&controller, 102);

    TEST_ASSERT_FALSE(controller.running);
}

void test_movement_action_rejects_invalid_values() {
    MovementActionController controller = {};

    TEST_ASSERT_EQUAL(
        ESP_ERR_INVALID_ARG,
        movement_action_controller_init(
            &controller, MOVEMENT_ACTION_GO_FORWARD, -0.1f));
    TEST_ASSERT_EQUAL(
        ESP_ERR_INVALID_ARG,
        movement_action_controller_init(
            &controller, MOVEMENT_ACTION_MAX, 1.0f));
    TEST_ASSERT_EQUAL(
        ESP_OK,
        movement_action_controller_init(
            &controller, MOVEMENT_ACTION_ROTATE, -90.0f));
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
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.25f, last_follow_speed_mps);
        TEST_ASSERT_EQUAL(
            static_cast<int>(StopCondition::DISTANCE),
            static_cast<int>(last_stop_condition));
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 0.75f, last_stop_value);
        TEST_ASSERT_FLOAT_WITHIN(0.001f, 12.0f, last_follow_timeout_s);
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

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_sequence_waits_for_arm_then_runs_square);
    RUN_TEST(test_non_odometry_frames_do_not_advance_movement_steps);
    RUN_TEST(test_arm_fault_stops_sequence);
    RUN_TEST(test_movement_action_rejects_invalid_values);
    RUN_TEST(test_tape_distance_actions_route_to_matching_sensor_direction);
    RUN_TEST(test_habitat_actions_have_unique_completion_details);
    return UNITY_END();
}
