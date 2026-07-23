/**
 * @file task_link_client.h
 * @brief Adapts a remote task endpoint to the local TaskActionExecutor API.
 *
 * The client sends start/cancel commands, retries unacknowledged commands,
 * tracks peer sessions and heartbeats, and exposes received status as a normal
 * executor result. It is used by drivetrain-to-arm and arm-to-Pi links.
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
    uint32_t heartbeat_interval_ms; /**< Period between requester heartbeats. */
    uint32_t link_timeout_ms; /**< Silence duration treated as peer failure. */
    uint32_t command_retry_ms; /**< Delay before resending an active command. */
    TaskEndpointId requester_endpoint; /**< Endpoint issuing commands. */
    TaskEndpointId executor_endpoint; /**< Expected remote endpoint. */
    uint32_t allowed_actions_mask; /**< Actions permitted on this link. */
} TaskLinkClientConfig;

typedef struct {
    uint32_t stale_status_count; /**< Ignored results for old identities. */
    uint32_t protocol_error_count; /**< Frames rejected as invalid. */
    uint32_t retry_count; /**< Commands retransmitted while awaiting status. */
} TaskLinkClientDiagnostics;

typedef struct {
    UartLink *link; /**< Framed UART transport owned by the application. */
    TaskLinkClientConfig config; /**< Validated timing and endpoint policy. */
    uint32_t requester_session_id; /**< This requester's boot identity. */
    uint32_t executor_session_id; /**< Most recently accepted peer identity. */
    uint32_t previous_executor_session_id; /**< Peer session rejected as stale. */
    uint32_t next_command_id; /**< Sequence number assigned to the next start. */
    uint32_t last_receive_ms; /**< Time of the last accepted peer message. */
    uint32_t last_heartbeat_ms; /**< Time this client last sent a heartbeat. */
    uint32_t last_command_send_ms; /**< Time the active command was last sent. */
    TaskFailure peer_failure; /**< Reset/link failure awaiting consumption. */
    bool command_active; /**< True while a remote command is tracked. */
    bool cancel_pending; /**< True until remote cancellation is confirmed. */
    TaskCommandMessage command; /**< Current correlated wire command. */
    TaskActionResult result; /**< Latest accepted remote result. */
    TaskLinkClientDiagnostics diagnostics; /**< Link health counters. */
} TaskLinkClient;

/** Initializes an idle client for one configured requester/executor pair. */
bool task_link_client_init(TaskLinkClient *client, UartLink *link,
                           uint32_t requester_session_id,
                           const TaskLinkClientConfig *config);
/** Accepts routed status and heartbeat frames from the UART packet router. */
void task_link_client_process_packet(void *context, const PacketFrame *frame,
                                     uint32_t now_ms);
/** Sends heartbeats/retries and detects remote link timeout. */
void task_link_client_update(TaskLinkClient *client, uint32_t now_ms);
/** Converts an underlying UART failure into a remote executor failure. */
void task_link_client_handle_link_error(TaskLinkClient *client);
/** Returns callbacks that let a coordinator or dispatcher use this client. */
TaskActionExecutor task_link_client_executor(TaskLinkClient *client);
/** Returns and clears a pending peer reset or link failure. */
bool task_link_client_take_peer_failure(TaskLinkClient *client,
                                        TaskFailure *failure_out);

#ifdef __cplusplus
}
#endif
