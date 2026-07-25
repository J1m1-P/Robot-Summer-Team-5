#include <string.h>

#include <unity.h>

extern "C" {
#include "control/task/robot_sequence_controller.h"
#include <robot_common/command_packet.h>
#include <robot_common/status_packet.h>
}

namespace {

uint32_t fake_millis = 0;
esp_err_t fake_uart_update_result = ESP_OK;
bool packet_queued = false;
PacketFrame queued_packet = {};
CommandOpcode sent_commands[16] = {};
size_t sent_command_count = 0;

const CommandOpcode kExpectedTowerCommands[] = {
    CMD_TOWER_HOME,
    CMD_TOWER_ROTATE_VERTICAL,
    CMD_TOWER_OPEN_CLAW,
    CMD_TOWER_Z_UP,
    CMD_TOWER_ROTATE_HORIZONTAL,
    CMD_TOWER_Z_DOWN,
    CMD_TOWER_CLOSE_CLAW,
    CMD_TOWER_Z_UP,
    CMD_TOWER_ROTATE_VERTICAL,
    CMD_TOWER_Z_UP,
};

void queue_status(StatusCode code, uint8_t detail) {
    queued_packet = {};
    queued_packet.message_type = PACKET_TYPE_STATUS;
    queued_packet.payload_len = STATUS_PACKET_PAYLOAD_SIZE;
    queued_packet.payload[0] = static_cast<uint8_t>(code);
    queued_packet.payload[1] = detail;
    packet_queued = true;
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

extern "C" esp_err_t uart_link_update(UartLink *) {
    return fake_uart_update_result;
}

extern "C" esp_err_t uart_link_take_packet(
    UartLink *,
    PacketFrame *packet_out) {
    if (!packet_queued) return ESP_ERR_NOT_FOUND;
    *packet_out = queued_packet;
    packet_queued = false;
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

extern "C" ActionStatusDetail tower_action_status_detail(
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
        default:
            return STATUS_DETAIL_NONE;
    }
}

void setUp() {
    fake_millis = 100;
    fake_uart_update_result = ESP_OK;
    packet_queued = false;
    queued_packet = {};
    memset(sent_commands, 0, sizeof(sent_commands));
    sent_command_count = 0;
}

void tearDown() {}

void test_sequence_runs_tape_then_tower_then_movement() {
    UartLink arm_uart = {};
    RobotSequenceController controller = {};

    TEST_ASSERT_EQUAL(
        ESP_OK,
        robot_sequence_controller_init(&controller, &arm_uart));
    TEST_ASSERT_EQUAL_UINT32(0, controller.current_step);
    TEST_ASSERT_EQUAL(
        MOVEMENT_ACTION_TAPE_FOLLOW_DISTANCE,
        controller.movement_action_controller.action);
    TEST_ASSERT_EQUAL_FLOAT(
        1.0f,
        controller.movement_action_controller.action_value);

    robot_sequence_controller_update(&controller, 100);
    TEST_ASSERT_EQUAL_UINT32(1, controller.current_step);
    TEST_ASSERT_EQUAL(
        MOVEMENT_ACTION_ROTATE_CW_UNTIL_TAPE_ALIGNED,
        controller.movement_action_controller.action);

    robot_sequence_controller_update(&controller, 101);
    TEST_ASSERT_EQUAL_UINT32(2, controller.current_step);
    TEST_ASSERT_EQUAL_UINT32(1, sent_command_count);
    TEST_ASSERT_EQUAL(kExpectedTowerCommands[0], sent_commands[0]);

    for (size_t index = 0;
         index < sizeof(kExpectedTowerCommands) /
             sizeof(kExpectedTowerCommands[0]);
         ++index) {
        const uint8_t mismatched_detail =
            static_cast<uint8_t>(STATUS_DETAIL_NONE);
        queue_status(STATUS_ACTION_COMPLETE, mismatched_detail);
        robot_sequence_controller_update(
            &controller,
            static_cast<uint32_t>(102 + index * 2));
        TEST_ASSERT_EQUAL_UINT32(2 + index, controller.current_step);

        queue_status(
            STATUS_ACTION_COMPLETE,
            static_cast<uint8_t>(
                tower_action_status_detail(kExpectedTowerCommands[index])));
        robot_sequence_controller_update(
            &controller,
            static_cast<uint32_t>(103 + index * 2));

        if (index + 1 < sizeof(kExpectedTowerCommands) /
                sizeof(kExpectedTowerCommands[0])) {
            TEST_ASSERT_EQUAL_UINT32(3 + index, controller.current_step);
            TEST_ASSERT_EQUAL_UINT32(index + 2, sent_command_count);
            TEST_ASSERT_EQUAL(
                kExpectedTowerCommands[index + 1],
                sent_commands[index + 1]);
        }
    }

    TEST_ASSERT_EQUAL_UINT32(12, controller.current_step);
    TEST_ASSERT_TRUE(controller.running);
    TEST_ASSERT_EQUAL(
        MOVEMENT_ACTION_GO_FORWARD,
        controller.movement_action_controller.action);
    TEST_ASSERT_EQUAL_FLOAT(
        1.0f,
        controller.movement_action_controller.action_value);

    robot_sequence_controller_update(&controller, 123);
    TEST_ASSERT_EQUAL_UINT32(13, controller.current_step);
    TEST_ASSERT_TRUE(controller.running);
    TEST_ASSERT_EQUAL(
        MOVEMENT_ACTION_ROTATE,
        controller.movement_action_controller.action);
    TEST_ASSERT_EQUAL_FLOAT(
        90.0f,
        controller.movement_action_controller.action_value);

    robot_sequence_controller_update(&controller, 124);
    TEST_ASSERT_EQUAL_UINT32(14, controller.current_step);
    TEST_ASSERT_FALSE(controller.running);
}

void test_arm_fault_stops_sequence() {
    UartLink arm_uart = {};
    RobotSequenceController controller = {};
    TEST_ASSERT_EQUAL(
        ESP_OK,
        robot_sequence_controller_init(&controller, &arm_uart));

    robot_sequence_controller_update(&controller, 100);
    robot_sequence_controller_update(&controller, 101);
    queue_status(STATUS_FAULT, STATUS_DETAIL_TOWER_HOME);
    robot_sequence_controller_update(&controller, 102);

    TEST_ASSERT_FALSE(controller.running);
}

void test_missing_arm_completion_times_out() {
    UartLink arm_uart = {};
    RobotSequenceController controller = {};
    TEST_ASSERT_EQUAL(
        ESP_OK,
        robot_sequence_controller_init(&controller, &arm_uart));

    robot_sequence_controller_update(&controller, 100);
    robot_sequence_controller_update(&controller, 101);
    robot_sequence_controller_update(&controller, 15102);

    TEST_ASSERT_FALSE(controller.running);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_sequence_runs_tape_then_tower_then_movement);
    RUN_TEST(test_arm_fault_stops_sequence);
    RUN_TEST(test_missing_arm_completion_times_out);
    return UNITY_END();
}
