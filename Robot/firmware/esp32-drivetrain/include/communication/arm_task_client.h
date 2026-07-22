/** @file arm_task_client.h
 *  @brief Drivetrain-side reliable client for arm-owned actions.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/task/task_protocol.h>
#include <robot_common/uart_link.h>

#include "task/task_action_executor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t heartbeat_interval_ms;
    uint32_t link_timeout_ms;
    uint32_t command_retry_ms;
} ArmTaskClientConfig;

typedef struct {
    uint32_t stale_status_count;
    uint32_t protocol_error_count;
    uint32_t retry_count;
} ArmTaskClientDiagnostics;

typedef struct {
    UartLink *link;
    ArmTaskClientConfig config;
    uint32_t coordinator_session_id;
    uint32_t arm_session_id;
    uint32_t previous_arm_session_id;
    uint32_t next_command_id;
    uint32_t last_receive_ms;
    uint32_t last_heartbeat_ms;
    uint32_t last_command_send_ms;
    bool peer_seen;
    bool peer_failure_pending;
    TaskFailure peer_failure;
    bool command_active;
    bool cancel_pending;
    TaskCommandMessage command;
    TaskActionResult result;
    ArmTaskClientDiagnostics diagnostics;
} ArmTaskClient;

bool arm_task_client_init(ArmTaskClient *client, UartLink *link,
                          uint32_t coordinator_session_id,
                          const ArmTaskClientConfig *config);
void arm_task_client_update_link(ArmTaskClient *client, uint32_t now_ms);
void arm_task_client_process_status(ArmTaskClient *client,
                                    const TaskStatusMessage *status,
                                    uint32_t now_ms);
TaskActionExecutor arm_task_client_executor(ArmTaskClient *client);
bool arm_task_client_is_connected(const ArmTaskClient *client);
bool arm_task_client_take_peer_failure(ArmTaskClient *client,
                                       TaskFailure *failure_out);
const ArmTaskClientDiagnostics *arm_task_client_get_diagnostics(
    const ArmTaskClient *client);

#ifdef __cplusplus
}
#endif
