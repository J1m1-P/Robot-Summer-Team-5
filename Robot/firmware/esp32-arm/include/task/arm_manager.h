/** @file arm_manager.h
 *  @brief Executes one arm-owned action without sequencing robot tasks.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/task/task.h>

#ifdef __cplusplus
extern "C" {
#endif

// Opaque: the concrete stepper/servo driver state is C++-only (Arduino and
// ESP32Servo types) and lives entirely in arm_manager.cpp, so this header --
// included by the plain-C task_server.c -- stays C-compatible.
typedef struct ArmMechanisms ArmMechanisms;

typedef struct {
    TaskStepStatus status;
    TaskFailure failure;
    TaskAction active_action;
    uint8_t current_step;
    uint32_t step_started_ms;
    ArmMechanisms *mechanisms;
} ArmManager;

// Initializes the arm's stepper/servo hardware once. A hardware failure is
// logged internally and leaves PICK_UP_BLOCK/BUILD_TOWER rejected rather
// than aborting arm boot entirely.
void arm_manager_init(ArmManager *manager);
bool arm_manager_start(ArmManager *manager, const TaskStepCommand *command,
                       uint32_t now_ms);
// Advances the active sequence's stepper motion and settle timing. Call
// every loop iteration; a no-op when no action is running.
void arm_manager_update(ArmManager *manager, uint32_t now_ms);
bool arm_manager_cancel(ArmManager *manager);
bool arm_manager_report_succeeded(ArmManager *manager);
bool arm_manager_report_failed(ArmManager *manager, TaskFailure failure);
TaskStepStatus arm_manager_get_status(const ArmManager *manager,
                                      TaskFailure *failure_out);

#ifdef __cplusplus
}
#endif
