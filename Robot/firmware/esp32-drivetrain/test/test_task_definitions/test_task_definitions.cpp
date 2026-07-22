#include <unity.h>

#include <robot_common/task/task_definition.h>

void setUp() {}
void tearDown() {}

void test_assigns_tower_steps_to_the_correct_controller() {
    TaskOwner owner{};

    TEST_ASSERT_TRUE(tower_building_step_get_owner(
        TOWER_BUILDING_STEP_MOVE_TO_TOWER, &owner));
    TEST_ASSERT_EQUAL(TASK_OWNER_DRIVETRAIN, owner);

    TEST_ASSERT_TRUE(tower_building_step_get_owner(
        TOWER_BUILDING_STEP_PICK_UP_BLOCK, &owner));
    TEST_ASSERT_EQUAL(TASK_OWNER_ARM, owner);

    TEST_ASSERT_TRUE(tower_building_step_get_owner(
        TOWER_BUILDING_STEP_PLACE_BLOCK, &owner));
    TEST_ASSERT_EQUAL(TASK_OWNER_ARM, owner);

    TEST_ASSERT_TRUE(tower_building_step_get_owner(
        TOWER_BUILDING_STEP_LEAVE, &owner));
    TEST_ASSERT_EQUAL(TASK_OWNER_DRIVETRAIN, owner);
}

void test_rejects_invalid_tower_steps() {
    TaskOwner owner{};

    TEST_ASSERT_FALSE(tower_building_step_get_owner(
        TOWER_BUILDING_STEP_COUNT, &owner));
    TEST_ASSERT_FALSE(tower_building_step_get_owner(
        TOWER_BUILDING_STEP_COUNT + 1U, &owner));
    TEST_ASSERT_FALSE(tower_building_step_get_owner(
        TOWER_BUILDING_STEP_MOVE_TO_TOWER, nullptr));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_assigns_tower_steps_to_the_correct_controller);
    RUN_TEST(test_rejects_invalid_tower_steps);
    return UNITY_END();
}
