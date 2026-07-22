#include <unity.h>

#include <robot_common/task/task_controller.h>
#include <robot_common/task/task_definition.h>

void setUp() {}
void tearDown() {}

void test_starts_each_tower_task_at_its_first_step() {
    TaskController controller{};
    task_controller_init(&controller);

    TEST_ASSERT_NULL(task_controller_get(&controller));
    TEST_ASSERT_TRUE(task_controller_start(
        &controller, TASK_TYPE_TOWER_PICKING));

    const Task *task = task_controller_get(&controller);
    TEST_ASSERT_NOT_NULL(task);
    TEST_ASSERT_EQUAL(TASK_OWNER_DRIVETRAIN, task->owner);
    TEST_ASSERT_EQUAL(TASK_TYPE_TOWER_PICKING, task->type);
    TEST_ASSERT_EQUAL(TOWER_PICKING_STEP_ALIGN_TO_PIECES,
                      task->current_step);

    TEST_ASSERT_TRUE(task_controller_cancel(&controller));
    TEST_ASSERT_TRUE(task_controller_start(
        &controller, TASK_TYPE_TOWER_BUILDING));

    task = task_controller_get(&controller);
    TEST_ASSERT_EQUAL(TASK_OWNER_DRIVETRAIN, task->owner);
    TEST_ASSERT_EQUAL(TASK_TYPE_TOWER_BUILDING, task->type);
    TEST_ASSERT_EQUAL(TOWER_BUILDING_STEP_ALIGN_TO_BASE, task->current_step);
    TEST_ASSERT_EQUAL(TASK_STATE_ACTIVE, task->state);
}

void test_advances_steps_and_completes_tower_picking() {
    TaskController controller{};
    task_controller_init(&controller);
    TEST_ASSERT_TRUE(task_controller_start(
        &controller, TASK_TYPE_TOWER_PICKING));

    TEST_ASSERT_TRUE(task_controller_step_succeeded(&controller));
    TEST_ASSERT_EQUAL(TOWER_PICKING_STEP_PICK_UP_BLOCK,
                      task_controller_get(&controller)->current_step);
    TEST_ASSERT_EQUAL(TASK_OWNER_ARM,
                      task_controller_get(&controller)->owner);

    TEST_ASSERT_TRUE(task_controller_step_succeeded(&controller));
    TEST_ASSERT_EQUAL(TOWER_PICKING_STEP_ALIGN_TO_TAPE,
                      task_controller_get(&controller)->current_step);
    TEST_ASSERT_EQUAL(TASK_OWNER_DRIVETRAIN,
                      task_controller_get(&controller)->owner);

    TEST_ASSERT_TRUE(task_controller_step_succeeded(&controller));
    TEST_ASSERT_EQUAL(TASK_STATE_COMPLETED,
                      task_controller_get(&controller)->state);
    TEST_ASSERT_FALSE(task_controller_step_succeeded(&controller));
}

void test_advances_steps_and_completes_tower_building() {
    TaskController controller{};
    task_controller_init(&controller);
    TEST_ASSERT_TRUE(task_controller_start(
        &controller, TASK_TYPE_TOWER_BUILDING));

    TEST_ASSERT_TRUE(task_controller_step_succeeded(&controller));
    TEST_ASSERT_EQUAL(TOWER_BUILDING_STEP_BUILD_TOWER,
                      task_controller_get(&controller)->current_step);
    TEST_ASSERT_EQUAL(TASK_OWNER_ARM,
                      task_controller_get(&controller)->owner);

    TEST_ASSERT_TRUE(task_controller_step_succeeded(&controller));
    TEST_ASSERT_EQUAL(TOWER_BUILDING_STEP_ALIGN_TO_TAPE,
                      task_controller_get(&controller)->current_step);
    TEST_ASSERT_EQUAL(TASK_OWNER_DRIVETRAIN,
                      task_controller_get(&controller)->owner);

    TEST_ASSERT_TRUE(task_controller_step_succeeded(&controller));
    TEST_ASSERT_EQUAL(TASK_STATE_COMPLETED,
                      task_controller_get(&controller)->state);
    TEST_ASSERT_FALSE(task_controller_step_succeeded(&controller));
}

void test_cancel_and_fault_end_active_tasks() {
    TaskController controller{};
    task_controller_init(&controller);
    TEST_ASSERT_TRUE(task_controller_start(
        &controller, TASK_TYPE_TOWER_BUILDING));
    TEST_ASSERT_TRUE(task_controller_cancel(&controller));
    TEST_ASSERT_EQUAL(TASK_STATE_CANCELLED,
                      task_controller_get(&controller)->state);
    TEST_ASSERT_FALSE(task_controller_fault(&controller));

    TEST_ASSERT_TRUE(task_controller_start(
        &controller, TASK_TYPE_TOWER_BUILDING));
    TEST_ASSERT_TRUE(task_controller_fault(&controller));
    TEST_ASSERT_EQUAL(TASK_STATE_FAULTED,
                      task_controller_get(&controller)->state);
}

void test_rejects_unsupported_or_overlapping_tasks() {
    TaskController controller{};
    task_controller_init(&controller);

    TEST_ASSERT_FALSE(task_controller_start(
        &controller, TASK_TYPE_TAPE_FOLLOWING));
    TEST_ASSERT_TRUE(task_controller_start(
        &controller, TASK_TYPE_TOWER_BUILDING));
    TEST_ASSERT_FALSE(task_controller_start(
        &controller, TASK_TYPE_TOWER_BUILDING));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_starts_each_tower_task_at_its_first_step);
    RUN_TEST(test_advances_steps_and_completes_tower_picking);
    RUN_TEST(test_advances_steps_and_completes_tower_building);
    RUN_TEST(test_cancel_and_fault_end_active_tasks);
    RUN_TEST(test_rejects_unsupported_or_overlapping_tasks);
    return UNITY_END();
}
