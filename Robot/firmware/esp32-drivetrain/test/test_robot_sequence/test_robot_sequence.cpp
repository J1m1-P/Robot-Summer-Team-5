#include <string.h>

#include <unity.h>

extern "C" {
#include "control/task/robot_sequence_controller.h"
#include <robot_common/command_packet.h>
#include <robot_common/pi_action_packet.h>
#include <robot_common/status_packet.h>
}

namespace {

uint32_t fake_millis = 0;
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
}

void tearDown() {}

void test_sequence_waits_for_arm_then_runs_enabled_tower_steps() {
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

    // Each placeholder tape-follow step completes, then a NOT_FOUND scan
    // sets the following rotation to zero before the next checkpoint.
    for (uint8_t scan = 0; scan < 3; ++scan) {
        const uint32_t tick = 102 + scan * 3;
        robot_sequence_controller_update(
            &controller, tick);
        TEST_ASSERT_EQUAL_UINT32(scan + 1, sent_command_count);
        TEST_ASSERT_EQUAL(CMD_PI_SCAN_TELETUBBIES, sent_commands[scan]);

        queue_pi_report(scan + 1, PI_RESULT_NOT_FOUND);
        deliver_frame(&controller, tick + 1);
        TEST_ASSERT_FLOAT_WITHIN(
            0.01f, 0.0f,
            controller.movement_action_controller.action_value);
        robot_sequence_controller_update(&controller, tick + 2);
    }

    TEST_ASSERT_EQUAL_UINT32(9, controller.current_step);
    TEST_ASSERT_EQUAL_UINT32(4, sent_command_count);
    TEST_ASSERT_EQUAL(kExpectedTowerCommands[0], sent_commands[3]);

    for (size_t index = 0;
         index < sizeof(kExpectedTowerCommands) /
             sizeof(kExpectedTowerCommands[0]);
         ++index) {
        const uint8_t mismatched_detail =
            static_cast<uint8_t>(STATUS_DETAIL_NONE);
        queue_status(STATUS_ACTION_COMPLETE, mismatched_detail);
        deliver_frame(&controller, static_cast<uint32_t>(111 + index * 2));
        TEST_ASSERT_EQUAL_UINT32(9 + index, controller.current_step);

        queue_status(
            STATUS_ACTION_COMPLETE,
            static_cast<uint8_t>(
                arm_action_status_detail(kExpectedTowerCommands[index])));
        deliver_frame(&controller, static_cast<uint32_t>(112 + index * 2));

        if (index + 1 < sizeof(kExpectedTowerCommands) /
                sizeof(kExpectedTowerCommands[0])) {
            TEST_ASSERT_EQUAL_UINT32(10 + index, controller.current_step);
            TEST_ASSERT_EQUAL_UINT32(index + 5, sent_command_count);
            TEST_ASSERT_EQUAL(
                kExpectedTowerCommands[index + 1],
                sent_commands[index + 4]);
        }
    }

    TEST_ASSERT_EQUAL_UINT32(19, controller.current_step);
    TEST_ASSERT_FALSE(controller.running);
}

void test_detected_target_sets_rotation_angle() {
    UartLink arm_uart = {};
    RobotSequenceController controller = {};
    TEST_ASSERT_EQUAL(
        ESP_OK,
        robot_sequence_controller_init(&controller, &arm_uart));

    queue_status(STATUS_ACTION_COMPLETE, STATUS_DETAIL_NONE);
    deliver_frame(&controller, 100);
    robot_sequence_controller_update(&controller, 101);
    TEST_ASSERT_EQUAL(CMD_PI_SCAN_TELETUBBIES, sent_commands[0]);

    queue_pi_report(1, PI_RESULT_OK, 0.5f);
    deliver_frame(&controller, 102);
    TEST_ASSERT_EQUAL(MOVEMENT_ACTION_ROTATE,
                      controller.movement_action_controller.action);
    TEST_ASSERT_FLOAT_WITHIN(
        0.01f, -22.5f,
        controller.movement_action_controller.action_value);

    robot_sequence_controller_update(&controller, 103);
    TEST_ASSERT_EQUAL_UINT32(3, controller.current_step);
    TEST_ASSERT_EQUAL_UINT32(1, sent_command_count);
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

void test_missing_arm_completion_times_out() {
    UartLink arm_uart = {};
    RobotSequenceController controller = {};
    TEST_ASSERT_EQUAL(
        ESP_OK,
        robot_sequence_controller_init(&controller, &arm_uart));

    queue_status(STATUS_ACTION_COMPLETE, STATUS_DETAIL_NONE);
    deliver_frame(&controller, 100);
    robot_sequence_controller_update(&controller, 101);
    robot_sequence_controller_update(&controller, 15102);

    TEST_ASSERT_FALSE(controller.running);
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
    RUN_TEST(test_sequence_waits_for_arm_then_runs_enabled_tower_steps);
    RUN_TEST(test_detected_target_sets_rotation_angle);
    RUN_TEST(test_arm_fault_stops_sequence);
    RUN_TEST(test_missing_arm_completion_times_out);
    RUN_TEST(test_habitat_actions_have_unique_completion_details);
    return UNITY_END();
}
