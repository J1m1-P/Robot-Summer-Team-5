/** @file task_link_server.h
 *  @brief Reliable server for actions requested by a remote task endpoint.
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
    uint32_t status_interval_ms;
    TaskEndpointId requester_endpoint;
    TaskEndpointId executor_endpoint;
} TaskLinkServerConfig;

typedef struct {
    uint32_t duplicate_command_count;
    uint32_t stale_command_count;
    uint32_t protocol_error_count;
} TaskLinkServerDiagnostics;

typedef struct {
    UartLink *link;
    TaskActionExecutor executor;
    TaskLinkServerConfig config;
    uint32_t executor_session_id;
    uint32_t requester_session_id;
    uint32_t previous_requester_session_id;
    uint32_t last_receive_ms;
    uint32_t last_heartbeat_ms;
    uint32_t last_status_ms;
    bool has_command;
    bool link_timed_out;
    TaskCommandMessage command;
    TaskActionResult result;
    TaskStepStatus last_sent_status;
    TaskLinkServerDiagnostics diagnostics;
} TaskLinkServer;

bool task_link_server_init(TaskLinkServer *server, UartLink *link,
                           const TaskActionExecutor *executor,
                           uint32_t executor_session_id,
                           const TaskLinkServerConfig *config);
void task_link_server_process_packet(void *context, const PacketFrame *frame,
                                     uint32_t now_ms);
void task_link_server_update(TaskLinkServer *server, uint32_t now_ms);
void task_link_server_handle_link_error(TaskLinkServer *server,
                                        uint32_t now_ms);

#ifdef __cplusplus
}
#endif
