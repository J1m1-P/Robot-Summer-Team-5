/** @file task_link_client.h
 *  @brief Reliable requester for actions executed by a remote task endpoint.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/task/task_action_executor.h>
#include <robot_common/task/task_protocol.h>
#include <robot_common/uart_link.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t heartbeat_interval_ms;
    uint32_t link_timeout_ms;
    uint32_t command_retry_ms;
    TaskEndpointId requester_endpoint;
    TaskEndpointId executor_endpoint;
    uint32_t allowed_actions_mask;
} TaskLinkClientConfig;

typedef struct {
    uint32_t stale_status_count;
    uint32_t protocol_error_count;
    uint32_t retry_count;
} TaskLinkClientDiagnostics;

typedef struct {
    UartLink *link;
    TaskLinkClientConfig config;
    uint32_t requester_session_id;
    uint32_t executor_session_id;
    uint32_t previous_executor_session_id;
    uint32_t next_command_id;
    uint32_t last_receive_ms;
    uint32_t last_heartbeat_ms;
    uint32_t last_command_send_ms;
    TaskFailure peer_failure;
    bool command_active;
    bool cancel_pending;
    TaskCommandMessage command;
    TaskActionResult result;
    TaskLinkClientDiagnostics diagnostics;
} TaskLinkClient;

bool task_link_client_init(TaskLinkClient *client, UartLink *link,
                           uint32_t requester_session_id,
                           const TaskLinkClientConfig *config);
void task_link_client_process_packet(void *context, const PacketFrame *frame,
                                     uint32_t now_ms);
void task_link_client_update(TaskLinkClient *client, uint32_t now_ms);
void task_link_client_handle_link_error(TaskLinkClient *client);
TaskActionExecutor task_link_client_executor(TaskLinkClient *client);
bool task_link_client_take_peer_failure(TaskLinkClient *client,
                                        TaskFailure *failure_out);

#ifdef __cplusplus
}
#endif
