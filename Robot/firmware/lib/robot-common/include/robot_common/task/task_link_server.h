/**
 * @file task_link_server.h
 * @brief Runs remote task commands through a local TaskActionExecutor.
 *
 * The server validates command identity and ordering, rejects overlapping or
 * stale work, polls the selected local executor, and publishes correlated
 * status and heartbeat frames back to the requester.
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
    uint32_t heartbeat_interval_ms; /**< Period between executor heartbeats. */
    uint32_t link_timeout_ms; /**< Silence duration that cancels active work. */
    uint32_t status_interval_ms; /**< Maximum delay between status reports. */
    TaskEndpointId requester_endpoint; /**< Expected command source. */
    TaskEndpointId executor_endpoint; /**< Identity advertised by this server. */
} TaskLinkServerConfig;

typedef struct {
    uint32_t duplicate_command_count; /**< Retries answered without restarting. */
    uint32_t stale_command_count; /**< Old commands rejected by identity/order. */
    uint32_t protocol_error_count; /**< Invalid routed frames received. */
} TaskLinkServerDiagnostics;

typedef struct {
    UartLink *link; /**< Framed UART transport owned by the application. */
    TaskActionExecutor executor; /**< Local action target for accepted commands. */
    TaskLinkServerConfig config; /**< Validated timing and endpoint policy. */
    uint32_t executor_session_id; /**< This executor's boot identity. */
    uint32_t requester_session_id; /**< Current accepted requester identity. */
    uint32_t previous_requester_session_id; /**< Requester session now considered stale. */
    uint32_t last_receive_ms; /**< Time of the last accepted requester message. */
    uint32_t last_heartbeat_ms; /**< Time this server last sent a heartbeat. */
    uint32_t last_status_ms; /**< Time this server last sent command status. */
    bool has_command; /**< True after accepting at least one current command. */
    bool link_timed_out; /**< Prevents repeating one timeout transition. */
    TaskCommandMessage command; /**< Current command and correlation values. */
    TaskActionResult result; /**< Latest local executor result. */
    TaskStepStatus last_sent_status; /**< Result state most recently transmitted. */
    TaskLinkServerDiagnostics diagnostics; /**< Link health counters. */
} TaskLinkServer;

/** Initializes an idle server around a local executor and UART link. */
bool task_link_server_init(TaskLinkServer *server, UartLink *link,
                           const TaskActionExecutor *executor,
                           uint32_t executor_session_id,
                           const TaskLinkServerConfig *config);
/** Accepts routed command and heartbeat frames from the packet router. */
void task_link_server_process_packet(void *context, const PacketFrame *frame,
                                     uint32_t now_ms);
/** Polls active work, publishes status, and enforces heartbeat timeouts. */
void task_link_server_update(TaskLinkServer *server, uint32_t now_ms);
/** Cancels active work and records timeout after an underlying UART error. */
void task_link_server_handle_link_error(TaskLinkServer *server,
                                        uint32_t now_ms);

#ifdef __cplusplus
}
#endif
