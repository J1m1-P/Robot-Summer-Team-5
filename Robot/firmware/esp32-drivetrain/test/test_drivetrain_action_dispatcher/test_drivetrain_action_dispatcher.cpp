#include <unity.h>

#include "task/drivetrain_action_dispatcher.h"

struct FakeAction {
    TaskActionResult result{TASK_STEP_NOT_STARTED, TASK_FAILURE_NONE};
    unsigned starts = 0U;
    unsigned updates = 0U;
    unsigned cancels = 0U;
    unsigned successes = 0U;
    unsigned failures = 0U;
    bool accept_start = true;
};

static bool fake_start(void *context, const TaskStepCommand *, uint32_t) {
    auto *action = static_cast<FakeAction *>(context);
    if (!action->accept_start) return false;
    action->starts++;
    action->result = {TASK_STEP_RUNNING, TASK_FAILURE_NONE};
    return true;
}

static TaskActionResult fake_update(void *context, uint32_t) {
    auto *action = static_cast<FakeAction *>(context);
    action->updates++;
    return action->result;
}

static void fake_cancel(void *context, uint32_t) {
    auto *action = static_cast<FakeAction *>(context);
    action->cancels++;
    action->result = {TASK_STEP_CANCELLED, TASK_FAILURE_NONE};
}

static bool fake_report_succeeded(void *context) {
    auto *action = static_cast<FakeAction *>(context);
    if (action->result.status != TASK_STEP_RUNNING) return false;
    action->successes++;
    action->result = {TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    return true;
}

static bool fake_report_failed(void *context, TaskFailure failure) {
    auto *action = static_cast<FakeAction *>(context);
    if (action->result.status != TASK_STEP_RUNNING ||
        failure == TASK_FAILURE_NONE) {
        return false;
    }
    action->failures++;
    action->result = {TASK_STEP_FAILED, failure};
    return true;
}

static DrivetrainActionHandler handler_for(FakeAction *action,
                                           uint32_t action_mask) {
    return {
        action_mask,
        {action, fake_start, fake_update, fake_cancel},
        fake_report_succeeded,
        fake_report_failed,
    };
}

static TaskStepCommand command_for(TaskAction action) {
    TaskStepCommand command{};
    command.action = action;
    return command;
}

void setUp() {}
void tearDown() {}

void test_routes_each_action_to_its_registered_handler() {
    DrivetrainActionDispatcher dispatcher{};
    drivetrain_action_dispatcher_init(&dispatcher);
    FakeAction follow{};
    FakeAction alignment{};
    const auto follow_handler = handler_for(
        &follow, TASK_ACTION_BIT(TASK_ACTION_FOLLOW_TAPE));
    const auto alignment_handler = handler_for(
        &alignment, TASK_ACTION_BIT(TASK_ACTION_ALIGN_TO_PIECES) |
                        TASK_ACTION_BIT(TASK_ACTION_ALIGN_TO_TAPE));
    TEST_ASSERT_TRUE(drivetrain_action_dispatcher_register(
        &dispatcher, &follow_handler));
    TEST_ASSERT_TRUE(drivetrain_action_dispatcher_register(
        &dispatcher, &alignment_handler));
    const auto executor = drivetrain_action_dispatcher_executor(&dispatcher);

    auto command = command_for(TASK_ACTION_ALIGN_TO_TAPE);
    TEST_ASSERT_TRUE(executor.start(executor.context, &command, 10U));
    TEST_ASSERT_EQUAL_UINT(0U, follow.starts);
    TEST_ASSERT_EQUAL_UINT(1U, alignment.starts);
}

void test_rejects_unknown_overlapping_and_busy_actions() {
    DrivetrainActionDispatcher dispatcher{};
    drivetrain_action_dispatcher_init(&dispatcher);
    FakeAction first{};
    FakeAction duplicate{};
    const auto first_handler = handler_for(
        &first, TASK_ACTION_BIT(TASK_ACTION_FOLLOW_TAPE));
    const auto duplicate_handler = handler_for(
        &duplicate, TASK_ACTION_BIT(TASK_ACTION_FOLLOW_TAPE));
    TEST_ASSERT_TRUE(drivetrain_action_dispatcher_register(
        &dispatcher, &first_handler));
    TEST_ASSERT_FALSE(drivetrain_action_dispatcher_register(
        &dispatcher, &duplicate_handler));
    const auto executor = drivetrain_action_dispatcher_executor(&dispatcher);

    auto unknown = command_for(TASK_ACTION_PICK_UP_BLOCK);
    TEST_ASSERT_FALSE(executor.start(executor.context, &unknown, 0U));

    auto follow = command_for(TASK_ACTION_FOLLOW_TAPE);
    TEST_ASSERT_TRUE(executor.start(executor.context, &follow, 0U));
    TEST_ASSERT_FALSE(executor.start(executor.context, &follow, 1U));
}

void test_terminal_update_releases_dispatcher_for_next_action() {
    DrivetrainActionDispatcher dispatcher{};
    drivetrain_action_dispatcher_init(&dispatcher);
    FakeAction action{};
    const auto handler = handler_for(
        &action, TASK_ACTION_BIT(TASK_ACTION_FOLLOW_TAPE));
    TEST_ASSERT_TRUE(
        drivetrain_action_dispatcher_register(&dispatcher, &handler));
    const auto executor = drivetrain_action_dispatcher_executor(&dispatcher);
    auto command = command_for(TASK_ACTION_FOLLOW_TAPE);

    TEST_ASSERT_TRUE(executor.start(executor.context, &command, 0U));
    action.result = {TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    TEST_ASSERT_EQUAL(TASK_STEP_SUCCEEDED,
                      executor.update(executor.context, 1U).status);
    TEST_ASSERT_TRUE(executor.start(executor.context, &command, 2U));
    TEST_ASSERT_EQUAL_UINT(2U, action.starts);
}

void test_cancel_and_external_results_reach_active_handler() {
    DrivetrainActionDispatcher dispatcher{};
    drivetrain_action_dispatcher_init(&dispatcher);
    FakeAction action{};
    const auto handler = handler_for(
        &action, TASK_ACTION_BIT(TASK_ACTION_FOLLOW_TAPE));
    TEST_ASSERT_TRUE(
        drivetrain_action_dispatcher_register(&dispatcher, &handler));
    const auto executor = drivetrain_action_dispatcher_executor(&dispatcher);
    auto command = command_for(TASK_ACTION_FOLLOW_TAPE);

    TEST_ASSERT_TRUE(executor.start(executor.context, &command, 0U));
    TEST_ASSERT_TRUE(drivetrain_action_dispatcher_report_failed(
        &dispatcher, TASK_FAILURE_STEP_FAILED));
    TEST_ASSERT_EQUAL_UINT(1U, action.failures);
    TEST_ASSERT_EQUAL(TASK_STEP_FAILED,
                      executor.update(executor.context, 1U).status);

    TEST_ASSERT_TRUE(executor.start(executor.context, &command, 2U));
    TEST_ASSERT_TRUE(
        drivetrain_action_dispatcher_report_succeeded(&dispatcher));
    TEST_ASSERT_EQUAL_UINT(1U, action.successes);
    TEST_ASSERT_EQUAL(TASK_STEP_SUCCEEDED,
                      executor.update(executor.context, 3U).status);

    TEST_ASSERT_TRUE(executor.start(executor.context, &command, 4U));
    executor.cancel(executor.context, 5U);
    TEST_ASSERT_EQUAL_UINT(1U, action.cancels);
    TEST_ASSERT_FALSE(
        drivetrain_action_dispatcher_report_succeeded(&dispatcher));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_routes_each_action_to_its_registered_handler);
    RUN_TEST(test_rejects_unknown_overlapping_and_busy_actions);
    RUN_TEST(test_terminal_update_releases_dispatcher_for_next_action);
    RUN_TEST(test_cancel_and_external_results_reach_active_handler);
    return UNITY_END();
}
