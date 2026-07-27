/* Declares the single arm_uart reader that keeps pose fusion continuously
 * up to date, independent of whatever else is running. */
#pragma once

#include "esp_err.h"

#include <robot_common/uart_link.h>

#include "comm/odometry_link.h"
#include "control/drivetrain/drivetrain.h"
#include "control/odometry/pose_tracker.h"
#include "control/task/robot_sequence_controller.h"

#ifdef __cplusplus
extern "C" {
#endif

// Every field is borrowed and must outlive the service.
typedef struct {
    PoseTracker *pose_tracker;
    Drivetrain *drivetrain;
    UartLink *arm_uart;
    Pmw3610OdometryLink *odometry_link;
    RobotSequenceController *sequence_controller;
} PoseService;

// Drains every frame currently queued on arm_uart -- routing odometry
// frames into odometry_link and everything else into sequence_controller --
// then advances pose_tracker with this cycle's wheel counts and the latest
// cached optical sample, if any. Call every control cycle from every loop
// that owns the drivetrain (main loop, blocking maneuvers like
// follow_tape()) so pose and arm_uart never stall behind whichever one
// currently has the CPU.
esp_err_t pose_service_update(PoseService *service, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
