#include <unity.h>

#include <robot_common/task/task_protocol.h>

void setUp() {}
void tearDown() {}

static TaskRequest tape_request(float distance_m, float speed_mps) {
    TaskRequest request{};
    request.type = TASK_TYPE_TAPE_FOLLOWING;
    request.step_parameters[0] = {distance_m, speed_mps, 0U};
    request.step_parameter_override_mask = UINT16_C(1);
    return request;
}

void test_request_validation_uses_the_same_step_parameters_for_every_task() {
    const TaskRequest forward = tape_request(1.0f, 0.2f);
    const TaskRequest reverse = tape_request(-1.0f, 0.2f);
    const TaskRequest no_distance = tape_request(0.0f, 0.2f);
    const TaskRequest no_speed = tape_request(1.0f, 0.0f);
    TEST_ASSERT_TRUE(task_request_is_valid(&forward));
    TEST_ASSERT_TRUE(task_request_is_valid(&reverse));
    TEST_ASSERT_FALSE(task_request_is_valid(&no_distance));
    TEST_ASSERT_FALSE(task_request_is_valid(&no_speed));

    TaskRequest picking{};
    picking.type = TASK_TYPE_TOWER_PICKING;
    TEST_ASSERT_TRUE(task_request_is_valid(&picking));
}

void test_reserved_action_is_not_executable() {
    TEST_ASSERT_TRUE(task_action_is_valid(TASK_ACTION_FOLLOW_TAPE));
    TEST_ASSERT_TRUE(task_action_is_valid(TASK_ACTION_BACK_OFF_PIECES));
    TEST_ASSERT_FALSE(task_action_is_valid(TASK_ACTION_RESERVED));
    TEST_ASSERT_FALSE(task_action_is_valid(TASK_ACTION_COUNT));
}

void test_command_protocol_round_trips_generic_step_parameters() {
    TaskCommandMessage command{};
    command.requester_session_id = 10U;
    command.command_id = 11U;
    command.type = TASK_COMMAND_START;
    command.step.execution_id = 12U;
    command.step.step = 4U;
    command.step.action = TASK_ACTION_POSITION_TOWER_X;
    command.step.parameters = {-0.125f, 0.5f, 250U};

    PacketFrame frame{};
    TEST_ASSERT_TRUE(task_protocol_encode_command(&command, &frame));
    TEST_ASSERT_EQUAL_UINT8(36U, frame.payload_len);

    TaskCommandMessage decoded{};
    TEST_ASSERT_TRUE(task_protocol_decode_command(&frame, &decoded));
    TEST_ASSERT_EQUAL(command.step.action, decoded.step.action);
    TEST_ASSERT_EQUAL_UINT8(command.step.step, decoded.step.step);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, command.step.parameters.amount,
                             decoded.step.parameters.amount);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, command.step.parameters.speed,
                             decoded.step.parameters.speed);
    TEST_ASSERT_EQUAL_UINT32(command.step.parameters.settle_ms,
                             decoded.step.parameters.settle_ms);

    command.step.action = TASK_ACTION_FOLLOW_TAPE;
    command.step.parameters = {-1.5f, 0.2f, 0U};
    TEST_ASSERT_TRUE(task_protocol_encode_command(&command, &frame));
    TEST_ASSERT_EQUAL_UINT8(1U, frame.payload[15]);
    TEST_ASSERT_TRUE(task_protocol_decode_command(&frame, &decoded));
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, -1.5f,
                             decoded.step.parameters.amount);
    TEST_ASSERT_FLOAT_WITHIN(0.00001f, 0.2f,
                             decoded.step.parameters.speed);
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(
        test_request_validation_uses_the_same_step_parameters_for_every_task);
    RUN_TEST(test_reserved_action_is_not_executable);
    RUN_TEST(test_command_protocol_round_trips_generic_step_parameters);
    return UNITY_END();
}
