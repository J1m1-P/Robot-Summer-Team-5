/** @file task_server.h
 *  @brief Arm-side reliable command server for drivetrain coordination.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/task/task_protocol.h>
#include <robot_common/uart_link.h>

#include "task/arm_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t heartbeat_interval_ms;
    uint32_t link_timeout_ms;
    uint32_t status_interval_ms;
} ArmTaskServerConfig;

typedef struct {
    uint32_t duplicate_command_count;
    uint32_t stale_command_count;
    uint32_t protocol_error_count;
} ArmTaskServerDiagnostics;

typedef struct {
    UartLink *link;
    ArmManager *manager;
    ArmTaskServerConfig config;
    uint32_t arm_session_id;
    uint32_t coordinator_session_id;
    uint32_t previous_coordinator_session_id;
    uint32_t last_receive_ms;
    uint32_t last_heartbeat_ms;
    uint32_t last_status_ms;
    bool coordinator_seen;
    bool has_command;
    TaskCommandMessage command;
    TaskStepStatus last_sent_status;
    ArmTaskServerDiagnostics diagnostics;
} ArmTaskServer;

bool arm_task_server_init(ArmTaskServer *server, UartLink *link,
                          ArmManager *manager, uint32_t arm_session_id,
                          const ArmTaskServerConfig *config);
void arm_task_server_update(ArmTaskServer *server, uint32_t now_ms);
void arm_task_server_process_command(ArmTaskServer *server,
                                     const TaskCommandMessage *command,
                                     uint32_t now_ms);
const ArmTaskServerDiagnostics *arm_task_server_get_diagnostics(
    const ArmTaskServer *server);

#ifdef __cplusplus
}
#endif
