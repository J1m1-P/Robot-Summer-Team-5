#include <unity.h>

#include <robot_common/task/task_definition.h>

#include "task/robot_task_manager.h"

void setUp() {}
void tearDown() {}

void test_starts_tower_task_owned_by_drivetrain() {
    RobotTaskManager manager{};
    robot_task_manager_init(&manager);

    TEST_ASSERT_NULL(robot_task_manager_get_task(&manager));
    TEST_ASSERT_TRUE(robot_task_manager_start_tower_building(&manager));

    const Task *task = robot_task_manager_get_task(&manager);
    TEST_ASSERT_NOT_NULL(task);
    TEST_ASSERT_EQUAL(TASK_TYPE_TOWER_BUILDING, task->type);
    TEST_ASSERT_EQUAL(TASK_OWNER_DRIVETRAIN, task->owner);
    TEST_ASSERT_EQUAL(TASK_STATE_ACTIVE, task->state);
}

void test_placeholder_action_keeps_current_step_running() {
    RobotTaskManager manager{};
    robot_task_manager_init(&manager);
    TEST_ASSERT_TRUE(robot_task_manager_start_tower_building(&manager));

    robot_task_manager_update(&manager);

    const Task *task = robot_task_manager_get_task(&manager);
    TEST_ASSERT_EQUAL(TASK_STATE_ACTIVE, task->state);
    TEST_ASSERT_EQUAL(TOWER_BUILDING_STEP_MOVE_TO_TOWER,
                      task->current_step);
}

void test_cancels_active_tower_task() {
    RobotTaskManager manager{};
    robot_task_manager_init(&manager);
    TEST_ASSERT_TRUE(robot_task_manager_start_tower_building(&manager));

    TEST_ASSERT_TRUE(robot_task_manager_cancel(&manager));
    TEST_ASSERT_EQUAL(TASK_STATE_CANCELLED,
                      robot_task_manager_get_task(&manager)->state);
    TEST_ASSERT_FALSE(robot_task_manager_cancel(&manager));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_tower_task_owned_by_drivetrain);
    RUN_TEST(test_placeholder_action_keeps_current_step_running);
    RUN_TEST(test_cancels_active_tower_task);
    return UNITY_END();
}
