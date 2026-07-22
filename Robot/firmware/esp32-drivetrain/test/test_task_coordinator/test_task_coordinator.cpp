#include <unity.h>

#include <cstring>

#include <robot_common/task/task_protocol.h>

#include "communication/task_server.h"
#include "communication/arm_task_client.h"
#include "task/task_coordinator.h"

struct FakeExecutor {
    TaskStepCommand command{};
    TaskActionResult result{TASK_STEP_RUNNING, TASK_FAILURE_NONE};
    unsigned starts = 0;
    unsigned cancels = 0;
    bool accept = true;
};

static bool fake_start(void *context, const TaskStepCommand *command,
                       uint32_t) {
    auto *fake = static_cast<FakeExecutor *>(context);
    if (!fake->accept) return false;
    fake->command = *command;
    fake->result = {TASK_STEP_RUNNING, TASK_FAILURE_NONE};
    fake->starts++;
    return true;
}

static TaskActionResult fake_update(void *context, uint32_t) {
    return static_cast<FakeExecutor *>(context)->result;
}

static void fake_cancel(void *context, uint32_t) {
    auto *fake = static_cast<FakeExecutor *>(context);
    fake->cancels++;
    fake->result = {TASK_STEP_CANCELLED, TASK_FAILURE_NONE};
}

static TaskActionExecutor executor_for(FakeExecutor *fake) {
    return {fake, fake_start, fake_update, fake_cancel};
}

static TaskRequest picking_request() {
    TaskRequest request{};
    request.type = TASK_TYPE_TOWER_PICKING;
    return request;
}

static TaskCommandMessage arm_command(uint32_t command_id) {
    TaskCommandMessage command{};
    command.coordinator_session_id = 10U;
    command.command_id = command_id;
    command.type = TASK_COMMAND_START;
    command.step.execution_id = 20U;
    command.step.step = 1U;
    command.step.action = TASK_ACTION_PICK_UP_BLOCK;
    return command;
}

extern "C" esp_err_t uart_link_update(UartLink *) { return ESP_OK; }
extern "C" esp_err_t uart_link_take_packet(UartLink *, PacketFrame *) {
    return ESP_ERR_NOT_FOUND;
}
extern "C" esp_err_t uart_link_send(UartLink *, PacketMessageType,
                                     const uint8_t *, uint8_t) {
    return ESP_OK;
}

void setUp() {}
void tearDown() {}

void test_successful_multi_step_task_has_one_authoritative_runtime() {
    FakeExecutor drivetrain{};
    FakeExecutor arm{};
    const auto drivetrain_executor = executor_for(&drivetrain);
    const auto arm_executor = executor_for(&arm);
    const TaskCoordinatorConfig config{1000U};
    TaskCoordinator coordinator{};
    TEST_ASSERT_TRUE(task_coordinator_init(&coordinator, &config,
                                           &drivetrain_executor,
                                           &arm_executor));
    const TaskRequest request = picking_request();
    TEST_ASSERT_TRUE(task_coordinator_start(&coordinator, &request, 42U, 0U));

    task_coordinator_update(&coordinator, 1U);
    TEST_ASSERT_EQUAL(TASK_ACTION_ALIGN_TO_PIECES, drivetrain.command.action);
    drivetrain.result = {TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    task_coordinator_update(&coordinator, 2U);
    TEST_ASSERT_EQUAL_UINT8(1U, coordinator.runtime.current_step);

    task_coordinator_update(&coordinator, 3U);
    TEST_ASSERT_EQUAL(TASK_ACTION_PICK_UP_BLOCK, arm.command.action);
    arm.result = {TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    task_coordinator_update(&coordinator, 4U);
    task_coordinator_update(&coordinator, 5U);
    drivetrain.result = {TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    task_coordinator_update(&coordinator, 6U);
    TEST_ASSERT_EQUAL(TASK_STATUS_SUCCEEDED, coordinator.runtime.status);
    TEST_ASSERT_EQUAL_UINT(2U, drivetrain.starts);
    TEST_ASSERT_EQUAL_UINT(1U, arm.starts);
}

void test_subsystem_failure_faults_task_without_advancing() {
    FakeExecutor drivetrain{};
    FakeExecutor arm{};
    const auto d = executor_for(&drivetrain);
    const auto a = executor_for(&arm);
    const TaskCoordinatorConfig config{1000U};
    TaskCoordinator coordinator{};
    task_coordinator_init(&coordinator, &config, &d, &a);
    const TaskRequest request = picking_request();
    task_coordinator_start(&coordinator, &request, 1U, 0U);
    task_coordinator_update(&coordinator, 1U);
    drivetrain.result = {TASK_STEP_FAILED, TASK_FAILURE_STEP_FAILED};
    task_coordinator_update(&coordinator, 2U);
    TEST_ASSERT_EQUAL(TASK_STATUS_FAILED, coordinator.runtime.status);
    TEST_ASSERT_EQUAL(TASK_FAILURE_STEP_FAILED, coordinator.runtime.failure);
    TEST_ASSERT_EQUAL_UINT8(0U, coordinator.runtime.current_step);
}

void test_step_timeout_cancels_executor_and_faults_task() {
    FakeExecutor drivetrain{};
    FakeExecutor arm{};
    const auto d = executor_for(&drivetrain);
    const auto a = executor_for(&arm);
    const TaskCoordinatorConfig config{50U};
    TaskCoordinator coordinator{};
    task_coordinator_init(&coordinator, &config, &d, &a);
    const TaskRequest request = picking_request();
    task_coordinator_start(&coordinator, &request, 1U, 0U);
    task_coordinator_update(&coordinator, 10U);
    task_coordinator_update(&coordinator, 60U);
    TEST_ASSERT_EQUAL(TASK_STATUS_FAILED, coordinator.runtime.status);
    TEST_ASSERT_EQUAL(TASK_FAILURE_STEP_TIMEOUT, coordinator.runtime.failure);
    TEST_ASSERT_EQUAL_UINT(1U, drivetrain.cancels);
}

void test_peer_reset_during_task_is_deterministic_failure() {
    FakeExecutor drivetrain{};
    FakeExecutor arm{};
    const auto d = executor_for(&drivetrain);
    const auto a = executor_for(&arm);
    const TaskCoordinatorConfig config{1000U};
    TaskCoordinator coordinator{};
    task_coordinator_init(&coordinator, &config, &d, &a);
    const TaskRequest request = picking_request();
    task_coordinator_start(&coordinator, &request, 1U, 0U);
    task_coordinator_update(&coordinator, 1U);
    TEST_ASSERT_TRUE(task_coordinator_fail(&coordinator,
                                           TASK_FAILURE_PEER_RESET, 2U));
    TEST_ASSERT_EQUAL(TASK_STATUS_FAILED, coordinator.runtime.status);
    TEST_ASSERT_EQUAL_UINT(1U, drivetrain.cancels);
}

void test_duplicate_arm_command_does_not_restart_action() {
    UartLink link{};
    ArmManager manager{};
    arm_manager_init(&manager);
    const ArmTaskServerConfig config{100U, 500U, 100U};
    ArmTaskServer server{};
    TEST_ASSERT_TRUE(arm_task_server_init(&server, &link, &manager, 30U,
                                          &config));
    const TaskCommandMessage command = arm_command(1U);
    arm_task_server_process_command(&server, &command, 1U);
    TEST_ASSERT_TRUE(manager.active);
    arm_task_server_process_command(&server, &command, 2U);
    TEST_ASSERT_TRUE(manager.active);
    TEST_ASSERT_EQUAL_UINT32(1U,
        server.diagnostics.duplicate_command_count);
}

void test_stale_command_and_status_are_rejected() {
    UartLink link{};
    ArmManager manager{};
    arm_manager_init(&manager);
    const ArmTaskServerConfig config{100U, 500U, 100U};
    ArmTaskServer server{};
    arm_task_server_init(&server, &link, &manager, 30U, &config);
    const TaskCommandMessage newer = arm_command(5U);
    arm_task_server_process_command(&server, &newer, 1U);
    arm_manager_report_succeeded(&manager);
    const TaskCommandMessage stale = arm_command(4U);
    arm_task_server_process_command(&server, &stale, 2U);
    TEST_ASSERT_EQUAL_UINT32(1U, server.diagnostics.stale_command_count);

    TaskStatusMessage status{10U, 30U, 20U, 5U,
                             TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    PacketFrame frame{};
    TEST_ASSERT_TRUE(task_protocol_encode_status(&status, &frame));
    TaskStatusMessage decoded{};
    TEST_ASSERT_TRUE(task_protocol_decode_status(&frame, &decoded));
    TEST_ASSERT_EQUAL_UINT32(5U, decoded.command_id);
    TEST_ASSERT_FALSE(task_protocol_sequence_is_newer(4U, 5U));
}

void test_client_rejects_stale_status_and_detects_link_timeout() {
    UartLink link{};
    const ArmTaskClientConfig config{100U, 500U, 100U};
    ArmTaskClient client{};
    TEST_ASSERT_TRUE(arm_task_client_init(&client, &link, 10U, &config));

    TaskStatusMessage unsolicited{10U, 30U, 99U, 8U,
                                  TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    arm_task_client_process_status(&client, &unsolicited, 1U);
    TEST_ASSERT_EQUAL_UINT32(1U, client.diagnostics.stale_status_count);
    TEST_ASSERT_TRUE(arm_task_client_is_connected(&client));

    arm_task_client_update_link(&client, 501U);
    TaskFailure failure = TASK_FAILURE_NONE;
    TEST_ASSERT_TRUE(arm_task_client_take_peer_failure(&client, &failure));
    TEST_ASSERT_EQUAL(TASK_FAILURE_LINK_TIMEOUT, failure);
    TEST_ASSERT_FALSE(arm_task_client_is_connected(&client));
}

void test_session_change_cancels_old_arm_work_and_rejects_old_session() {
    UartLink link{};
    ArmManager manager{};
    arm_manager_init(&manager);
    const ArmTaskServerConfig config{100U, 500U, 100U};
    ArmTaskServer server{};
    arm_task_server_init(&server, &link, &manager, 30U, &config);

    const TaskCommandMessage old_command = arm_command(1U);
    arm_task_server_process_command(&server, &old_command, 1U);
    TEST_ASSERT_TRUE(manager.active);

    TaskCommandMessage reset_command = arm_command(1U);
    reset_command.coordinator_session_id = 11U;
    reset_command.step.execution_id = 21U;
    arm_task_server_process_command(&server, &reset_command, 2U);
    TEST_ASSERT_TRUE(manager.active);
    TEST_ASSERT_EQUAL_UINT32(10U, server.previous_coordinator_session_id);

    arm_task_server_process_command(&server, &old_command, 3U);
    TEST_ASSERT_TRUE(manager.active);
    TEST_ASSERT_EQUAL_UINT32(1U, server.diagnostics.stale_command_count);
    TEST_ASSERT_EQUAL_UINT32(11U, server.coordinator_session_id);
}

void test_rejects_overlapping_task_and_invalid_request() {
    FakeExecutor drivetrain{};
    FakeExecutor arm{};
    const auto d = executor_for(&drivetrain);
    const auto a = executor_for(&arm);
    const TaskCoordinatorConfig config{1000U};
    TaskCoordinator coordinator{};
    task_coordinator_init(&coordinator, &config, &d, &a);
    const TaskRequest request = picking_request();
    TEST_ASSERT_TRUE(task_coordinator_start(&coordinator, &request, 1U, 0U));
    TEST_ASSERT_FALSE(task_coordinator_start(&coordinator, &request, 2U, 0U));
    TaskRequest invalid{};
    invalid.type = TASK_TYPE_COUNT;
    TaskCoordinator idle{};
    task_coordinator_init(&idle, &config, &d, &a);
    TEST_ASSERT_FALSE(task_coordinator_start(&idle, &invalid, 1U, 0U));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_successful_multi_step_task_has_one_authoritative_runtime);
    RUN_TEST(test_subsystem_failure_faults_task_without_advancing);
    RUN_TEST(test_step_timeout_cancels_executor_and_faults_task);
    RUN_TEST(test_peer_reset_during_task_is_deterministic_failure);
    RUN_TEST(test_duplicate_arm_command_does_not_restart_action);
    RUN_TEST(test_stale_command_and_status_are_rejected);
    RUN_TEST(test_client_rejects_stale_status_and_detects_link_timeout);
    RUN_TEST(test_session_change_cancels_old_arm_work_and_rejects_old_session);
    RUN_TEST(test_rejects_overlapping_task_and_invalid_request);
    return UNITY_END();
}
