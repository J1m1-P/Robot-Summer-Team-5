#include <robot_common/task/task_link_server.h>

#include <stddef.h>
#include <string.h>

static bool elapsed_at_least(uint32_t now, uint32_t then, uint32_t duration) {
    return (uint32_t)(now - then) >= duration;
}

static bool executor_is_valid(const TaskActionExecutor *executor) {
    return executor != NULL && executor->start != NULL &&
           executor->update != NULL && executor->cancel != NULL;
}

static bool send_frame(UartLink *link, const PacketFrame *frame) {
    return link != NULL && frame != NULL &&
           uart_link_send(link, (PacketMessageType)frame->message_type,
                          frame->payload, frame->payload_len) == ESP_OK;
}

static bool send_heartbeat(TaskLinkServer *server) {
    const TaskHeartbeatMessage heartbeat = {
        .sender = server->config.executor_endpoint,
        .session_id = server->executor_session_id,
    };
    PacketFrame frame = {0};
    return task_protocol_encode_heartbeat(&heartbeat, &frame) &&
           send_frame(server->link, &frame);
}

static bool send_status_for(TaskLinkServer *server,
                            const TaskCommandMessage *command,
                            TaskActionResult result) {
    const TaskStatusMessage message = {
        .requester_session_id = command->requester_session_id,
        .executor_session_id = server->executor_session_id,
        .execution_id = command->step.execution_id,
        .command_id = command->command_id,
        .status = result.status,
        .failure = result.failure,
    };
    PacketFrame frame = {0};
    return task_protocol_encode_status(&message, &frame) &&
           send_frame(server->link, &frame);
}

static void cancel_active(TaskLinkServer *server, uint32_t now_ms) {
    if (server->has_command && server->result.status == TASK_STEP_RUNNING) {
        server->executor.cancel(server->executor.context, now_ms);
    }
    server->has_command = false;
}

static void reset_for_requester(TaskLinkServer *server,
                                uint32_t requester_session_id,
                                uint32_t now_ms) {
    if (server->requester_session_id != 0U &&
        server->requester_session_id != requester_session_id) {
        cancel_active(server, now_ms);
        server->previous_requester_session_id = server->requester_session_id;
    }
    server->requester_session_id = requester_session_id;
    server->has_command = false;
    server->link_timed_out = false;
    server->last_receive_ms = now_ms;
}

static void process_command(TaskLinkServer *server,
                            const TaskCommandMessage *command,
                            uint32_t now_ms) {
    if (server->requester_session_id != 0U &&
        command->requester_session_id ==
            server->previous_requester_session_id) {
        server->diagnostics.stale_command_count++;
        (void)send_status_for(server, command,
                              (TaskActionResult){TASK_STEP_FAILED,
                                                 TASK_FAILURE_STALE_MESSAGE});
        return;
    }
    if (server->requester_session_id == 0U ||
        server->requester_session_id != command->requester_session_id) {
        reset_for_requester(server, command->requester_session_id, now_ms);
    }
    server->last_receive_ms = now_ms;
    server->link_timed_out = false;

    if (server->has_command &&
        command->step.execution_id == server->command.step.execution_id &&
        command->command_id == server->command.command_id) {
        server->diagnostics.duplicate_command_count++;
        if (command->type == TASK_COMMAND_CANCEL &&
            server->result.status == TASK_STEP_RUNNING) {
            server->executor.cancel(server->executor.context, now_ms);
            server->result = server->executor.update(server->executor.context,
                                                      now_ms);
        }
        (void)send_status_for(server, command, server->result);
        return;
    }
    if (server->has_command &&
        !task_protocol_sequence_is_newer(command->command_id,
                                         server->command.command_id)) {
        server->diagnostics.stale_command_count++;
        (void)send_status_for(server, command,
                              (TaskActionResult){TASK_STEP_FAILED,
                                                 TASK_FAILURE_STALE_MESSAGE});
        return;
    }
    if (command->type == TASK_COMMAND_CANCEL) {
        server->diagnostics.stale_command_count++;
        (void)send_status_for(server, command,
                              (TaskActionResult){TASK_STEP_FAILED,
                                                 TASK_FAILURE_STALE_MESSAGE});
        return;
    }
    if (server->has_command && server->result.status == TASK_STEP_RUNNING) {
        (void)send_status_for(server, command,
                              (TaskActionResult){TASK_STEP_FAILED,
                                                 TASK_FAILURE_BUSY});
        return;
    }

    server->command = *command;
    server->has_command = true;
    if (!server->executor.start(server->executor.context, &command->step,
                                now_ms)) {
        server->result = (TaskActionResult){TASK_STEP_FAILED,
                                            TASK_FAILURE_STEP_REJECTED};
        (void)send_status_for(server, command, server->result);
        return;
    }
    server->result = (TaskActionResult){TASK_STEP_RUNNING, TASK_FAILURE_NONE};
    server->last_sent_status = TASK_STEP_RUNNING;
    server->last_status_ms = now_ms;
    (void)send_status_for(server, command, server->result);
}

bool task_link_server_init(TaskLinkServer *server, UartLink *link,
                           const TaskActionExecutor *executor,
                           uint32_t executor_session_id,
                           const TaskLinkServerConfig *config) {
    if (server == NULL || link == NULL || !executor_is_valid(executor) ||
        config == NULL || executor_session_id == 0U ||
        config->heartbeat_interval_ms == 0U ||
        config->link_timeout_ms <= config->heartbeat_interval_ms ||
        config->status_interval_ms == 0U ||
        config->requester_endpoint >= TASK_ENDPOINT_COUNT ||
        config->executor_endpoint >= TASK_ENDPOINT_COUNT ||
        config->requester_endpoint == config->executor_endpoint) {
        return false;
    }
    memset(server, 0, sizeof(*server));
    server->link = link;
    server->executor = *executor;
    server->config = *config;
    server->executor_session_id = executor_session_id;
    server->result.status = TASK_STEP_NOT_STARTED;
    server->last_sent_status = TASK_STEP_NOT_STARTED;
    return true;
}

void task_link_server_process_packet(void *context, const PacketFrame *frame,
                                     uint32_t now_ms) {
    TaskLinkServer *server = (TaskLinkServer *)context;
    if (server == NULL || frame == NULL) return;

    TaskHeartbeatMessage heartbeat = {0};
    TaskCommandMessage command = {0};
    if (task_protocol_decode_heartbeat(frame, &heartbeat) &&
        heartbeat.sender == server->config.requester_endpoint) {
        if (server->requester_session_id != 0U &&
            heartbeat.session_id == server->previous_requester_session_id) {
            return;
        }
        if (server->requester_session_id == 0U ||
            server->requester_session_id != heartbeat.session_id) {
            reset_for_requester(server, heartbeat.session_id, now_ms);
        } else {
            server->last_receive_ms = now_ms;
            server->link_timed_out = false;
        }
    } else if (task_protocol_decode_command(frame, &command)) {
        process_command(server, &command, now_ms);
    } else {
        server->diagnostics.protocol_error_count++;
    }
}

void task_link_server_update(TaskLinkServer *server, uint32_t now_ms) {
    if (server == NULL || server->link == NULL) return;
    if (server->last_heartbeat_ms == 0U ||
        elapsed_at_least(now_ms, server->last_heartbeat_ms,
                         server->config.heartbeat_interval_ms)) {
        (void)send_heartbeat(server);
        server->last_heartbeat_ms = now_ms;
    }
    if (server->requester_session_id != 0U && !server->link_timed_out &&
        elapsed_at_least(now_ms, server->last_receive_ms,
                         server->config.link_timeout_ms)) {
        if (server->has_command && server->result.status == TASK_STEP_RUNNING) {
            server->executor.cancel(server->executor.context, now_ms);
            server->result = (TaskActionResult){TASK_STEP_FAILED,
                                                TASK_FAILURE_LINK_TIMEOUT};
        }
        server->link_timed_out = true;
    }
    if (server->has_command) {
        if (server->result.status == TASK_STEP_RUNNING) {
            server->result = server->executor.update(server->executor.context,
                                                      now_ms);
        }
        if (server->result.status != server->last_sent_status ||
            elapsed_at_least(now_ms, server->last_status_ms,
                             server->config.status_interval_ms)) {
            (void)send_status_for(server, &server->command, server->result);
            server->last_sent_status = server->result.status;
            server->last_status_ms = now_ms;
        }
    }
}

void task_link_server_handle_link_error(TaskLinkServer *server,
                                        uint32_t now_ms) {
    if (server == NULL) return;
    if (server->has_command && server->result.status == TASK_STEP_RUNNING) {
        server->executor.cancel(server->executor.context, now_ms);
        server->result = (TaskActionResult){TASK_STEP_FAILED,
                                            TASK_FAILURE_LINK_TIMEOUT};
    }
    server->link_timed_out = true;
}
