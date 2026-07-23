#include <unity.h>

#include "task/drivetrain_manager.h"

static esp_err_t hardware_init_result = ESP_OK;
static unsigned follow_starts = 0U;
static unsigned follow_updates = 0U;
static unsigned follow_cancels = 0U;
static unsigned alignment_starts = 0U;
static unsigned alignment_updates = 0U;
static unsigned alignment_cancels = 0U;

esp_err_t tape_sensor_mux_init(TapeSensorMux *,
                               const TapeSensorMuxConfig *) {
    return hardware_init_result;
}

esp_err_t tape_sensor_driver_init(TapeSensor *,
                                  const TapeSensorDriverConfig *,
                                  TapeSensorMux *) {
    return hardware_init_result;
}

void follow_tape_action_init(FollowTapeAction *action, Drivetrain *,
                             TapeSensor *, TapeSensor *, TapeSensor *,
                             TapeFollower *) {
    action->result = {TASK_STEP_NOT_STARTED, TASK_FAILURE_NONE};
}

bool follow_tape_action_start(FollowTapeAction *action,
                              const TaskStepCommand *command, uint32_t) {
    if (command->action != TASK_ACTION_FOLLOW_TAPE ||
        action->result.status == TASK_STEP_RUNNING) {
        return false;
    }
    follow_starts++;
    action->result = {TASK_STEP_RUNNING, TASK_FAILURE_NONE};
    return true;
}

TaskActionResult follow_tape_action_update(FollowTapeAction *action,
                                           uint32_t) {
    follow_updates++;
    return action->result;
}

void follow_tape_action_cancel(FollowTapeAction *action) {
    follow_cancels++;
    action->result = {TASK_STEP_CANCELLED, TASK_FAILURE_NONE};
}

bool follow_tape_action_report_succeeded(FollowTapeAction *action) {
    if (action->result.status != TASK_STEP_RUNNING) return false;
    action->result = {TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    return true;
}

bool follow_tape_action_report_failed(FollowTapeAction *action,
                                      TaskFailure failure) {
    if (action->result.status != TASK_STEP_RUNNING ||
        failure == TASK_FAILURE_NONE) {
        return false;
    }
    action->result = {TASK_STEP_FAILED, failure};
    return true;
}

void tape_alignment_action_init(TapeAlignmentAction *action, Drivetrain *,
                                TapeSensor *, TapeSensor *, TapeSensor *,
                                TapeFollower *) {
    action->result = {TASK_STEP_NOT_STARTED, TASK_FAILURE_NONE};
}

bool tape_alignment_action_start(TapeAlignmentAction *action,
                                 const TaskStepCommand *command, uint32_t) {
    if (command->action == TASK_ACTION_FOLLOW_TAPE ||
        action->result.status == TASK_STEP_RUNNING) {
        return false;
    }
    alignment_starts++;
    action->active_action = command->action;
    action->result = {TASK_STEP_RUNNING, TASK_FAILURE_NONE};
    return true;
}

TaskActionResult tape_alignment_action_update(TapeAlignmentAction *action,
                                              uint32_t) {
    alignment_updates++;
    return action->result;
}

void tape_alignment_action_cancel(TapeAlignmentAction *action) {
    alignment_cancels++;
    action->result = {TASK_STEP_CANCELLED, TASK_FAILURE_NONE};
}

bool tape_alignment_action_report_succeeded(TapeAlignmentAction *action) {
    if (action->result.status != TASK_STEP_RUNNING) return false;
    action->result = {TASK_STEP_SUCCEEDED, TASK_FAILURE_NONE};
    return true;
}

bool tape_alignment_action_report_failed(TapeAlignmentAction *action,
                                         TaskFailure failure) {
    if (action->result.status != TASK_STEP_RUNNING ||
        failure == TASK_FAILURE_NONE) {
        return false;
    }
    action->result = {TASK_STEP_FAILED, failure};
    return true;
}

static DrivetrainManager make_manager() {
    DrivetrainManager manager{};
    manager.active_action = TASK_ACTION_COUNT;
    manager.tape_hardware_ready = true;
    follow_tape_action_init(&manager.follow_tape, nullptr, nullptr, nullptr,
                            nullptr, nullptr);
    tape_alignment_action_init(&manager.tape_alignment, nullptr, nullptr,
                               nullptr, nullptr, nullptr);
    return manager;
}

static TaskStepCommand command_for(TaskAction action) {
    TaskStepCommand command{};
    command.action = action;
    return command;
}

void setUp() {
    hardware_init_result = ESP_OK;
    follow_starts = 0U;
    follow_updates = 0U;
    follow_cancels = 0U;
    alignment_starts = 0U;
    alignment_updates = 0U;
    alignment_cancels = 0U;
}

void tearDown() {}

void test_routes_follow_and_alignment_actions_directly() {
    DrivetrainManager manager = make_manager();
    const TaskActionExecutor executor = drivetrain_manager_executor(&manager);

    TaskStepCommand follow = command_for(TASK_ACTION_FOLLOW_TAPE);
    TEST_ASSERT_TRUE(executor.start(executor.context, &follow, 1U));
    TEST_ASSERT_EQUAL_UINT(1U, follow_starts);
    TEST_ASSERT_EQUAL_UINT(0U, alignment_starts);
    executor.cancel(executor.context, 2U);

    const TaskAction alignment_actions[] = {
        TASK_ACTION_ALIGN_TO_PIECES,
        TASK_ACTION_FOLLOW_PIECES_TAPE,
        TASK_ACTION_FOLLOW_TASK_TAPE,
        TASK_ACTION_BACK_OFF_PIECES,
        TASK_ACTION_ALIGN_TO_TAPE,
    };
    for (TaskAction action : alignment_actions) {
        TaskStepCommand command = command_for(action);
        TEST_ASSERT_TRUE(executor.start(executor.context, &command, 3U));
        executor.cancel(executor.context, 4U);
    }
    TEST_ASSERT_EQUAL_UINT(5U, alignment_starts);
}

void test_rejects_busy_unsupported_and_uninitialized_actions() {
    DrivetrainManager manager = make_manager();
    const TaskActionExecutor executor = drivetrain_manager_executor(&manager);
    TaskStepCommand follow = command_for(TASK_ACTION_FOLLOW_TAPE);
    TaskStepCommand top = command_for(TASK_ACTION_PICK_UP_BLOCK);

    TEST_ASSERT_FALSE(executor.start(executor.context, &top, 0U));
    TEST_ASSERT_TRUE(executor.start(executor.context, &follow, 1U));
    TEST_ASSERT_FALSE(executor.start(executor.context, &follow, 2U));

    hardware_init_result = ESP_ERR_INVALID_STATE;
    DrivetrainManager unavailable{};
    TEST_ASSERT_EQUAL(
        ESP_ERR_INVALID_STATE,
        drivetrain_manager_init(&unavailable, nullptr, nullptr, nullptr,
                                nullptr, nullptr, nullptr));
    const TaskActionExecutor unavailable_executor =
        drivetrain_manager_executor(&unavailable);
    TEST_ASSERT_FALSE(unavailable_executor.start(
        unavailable_executor.context, &follow, 3U));
}

void test_terminal_update_releases_manager_for_next_action() {
    DrivetrainManager manager = make_manager();
    const TaskActionExecutor executor = drivetrain_manager_executor(&manager);
    TaskStepCommand follow = command_for(TASK_ACTION_FOLLOW_TAPE);

    TEST_ASSERT_TRUE(executor.start(executor.context, &follow, 0U));
    TEST_ASSERT_TRUE(drivetrain_manager_report_succeeded(&manager));
    TEST_ASSERT_EQUAL(TASK_STEP_SUCCEEDED,
                      executor.update(executor.context, 1U).status);
    TEST_ASSERT_EQUAL(TASK_ACTION_COUNT, manager.active_action);
    TEST_ASSERT_TRUE(executor.start(executor.context, &follow, 2U));
    TEST_ASSERT_EQUAL_UINT(2U, follow_starts);
}

void test_external_results_and_cancel_reach_active_action() {
    DrivetrainManager manager = make_manager();
    const TaskActionExecutor executor = drivetrain_manager_executor(&manager);
    TaskStepCommand alignment = command_for(TASK_ACTION_ALIGN_TO_TAPE);

    TEST_ASSERT_TRUE(executor.start(executor.context, &alignment, 0U));
    TEST_ASSERT_TRUE(drivetrain_manager_report_failed(
        &manager, TASK_FAILURE_STEP_FAILED));
    TEST_ASSERT_EQUAL(TASK_STEP_FAILED,
                      executor.update(executor.context, 1U).status);
    TEST_ASSERT_EQUAL_UINT(1U, alignment_updates);

    TEST_ASSERT_TRUE(executor.start(executor.context, &alignment, 2U));
    executor.cancel(executor.context, 3U);
    TEST_ASSERT_EQUAL_UINT(1U, alignment_cancels);
    TEST_ASSERT_EQUAL(TASK_ACTION_COUNT, manager.active_action);
    TEST_ASSERT_FALSE(drivetrain_manager_report_succeeded(&manager));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_routes_follow_and_alignment_actions_directly);
    RUN_TEST(test_rejects_busy_unsupported_and_uninitialized_actions);
    RUN_TEST(test_terminal_update_releases_manager_for_next_action);
    RUN_TEST(test_external_results_and_cancel_reach_active_action);
    return UNITY_END();
}
