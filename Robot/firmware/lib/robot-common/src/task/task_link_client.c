#include <robot_common/task/task_link_client.h>

#include <stddef.h>
#include <string.h>

static bool elapsed_at_least(uint32_t now, uint32_t then, uint32_t duration) {
    return (uint32_t)(now - then) >= duration;
}

static bool send_frame(UartLink *link, const PacketFrame *frame) {
    return link != NULL && frame != NULL &&
           uart_link_send(link, (PacketMessageType)frame->message_type,
                          frame->payload, frame->payload_len) == ESP_OK;
}

static bool send_heartbeat(TaskLinkClient *client) {
    const TaskHeartbeatMessage heartbeat = {
        .sender = client->config.requester_endpoint,
        .session_id = client->requester_session_id,
    };
    PacketFrame frame = {0};
    return task_protocol_encode_heartbeat(&heartbeat, &frame) &&
           send_frame(client->link, &frame);
}

static bool send_command(TaskLinkClient *client, TaskCommandType type) {
    TaskCommandMessage message = client->command;
    message.type = type;
    PacketFrame frame = {0};
    return task_protocol_encode_command(&message, &frame) &&
           send_frame(client->link, &frame);
}

static void record_peer_failure(TaskLinkClient *client, TaskFailure failure) {
    client->peer_failure = failure;
    if (client->command_active) {
        client->result.status = TASK_STEP_FAILED;
        client->result.failure = failure;
    }
}

static bool accept_executor_session(TaskLinkClient *client,
                                    uint32_t session_id, uint32_t now_ms) {
    const bool reset = client->executor_session_id != 0U &&
                       client->executor_session_id != session_id;
    if (reset) {
        if (session_id == client->previous_executor_session_id) {
            client->diagnostics.stale_status_count++;
            return true;
        }
        client->previous_executor_session_id = client->executor_session_id;
        record_peer_failure(client, TASK_FAILURE_PEER_RESET);
        client->cancel_pending = false;
    }
    client->executor_session_id = session_id;
    client->last_receive_ms = now_ms;
    return reset;
}

static void process_status(TaskLinkClient *client,
                           const TaskStatusMessage *status,
                           uint32_t now_ms) {
    if (status->requester_session_id != client->requester_session_id) {
        client->diagnostics.stale_status_count++;
        return;
    }
    if (accept_executor_session(client, status->executor_session_id, now_ms)) {
        return;
    }
    if (!client->command_active ||
        status->execution_id != client->command.step.execution_id ||
        status->command_id != client->command.command_id) {
        client->diagnostics.stale_status_count++;
        return;
    }
    client->result.status = status->status;
    client->result.failure = status->failure;
    if (client->cancel_pending && task_step_status_is_terminal(status->status)) {
        client->command_active = false;
        client->cancel_pending = false;
    }
}

bool task_link_client_init(TaskLinkClient *client, UartLink *link,
                           uint32_t requester_session_id,
                           const TaskLinkClientConfig *config) {
    if (client == NULL || link == NULL || config == NULL ||
        requester_session_id == 0U || config->heartbeat_interval_ms == 0U ||
        config->link_timeout_ms <= config->heartbeat_interval_ms ||
        config->command_retry_ms == 0U ||
        config->requester_endpoint >= TASK_ENDPOINT_COUNT ||
        config->executor_endpoint >= TASK_ENDPOINT_COUNT ||
        config->requester_endpoint == config->executor_endpoint ||
        config->allowed_actions_mask == 0U) {
        return false;
    }
    memset(client, 0, sizeof(*client));
    client->link = link;
    client->config = *config;
    client->requester_session_id = requester_session_id;
    client->next_command_id = 1U;
    client->result.status = TASK_STEP_NOT_STARTED;
    return true;
}

void task_link_client_process_packet(void *context, const PacketFrame *frame,
                                     uint32_t now_ms) {
    TaskLinkClient *client = (TaskLinkClient *)context;
    if (client == NULL || frame == NULL) return;

    TaskHeartbeatMessage heartbeat = {0};
    TaskStatusMessage status = {0};
    if (task_protocol_decode_heartbeat(frame, &heartbeat) &&
        heartbeat.sender == client->config.executor_endpoint) {
        (void)accept_executor_session(client, heartbeat.session_id, now_ms);
    } else if (task_protocol_decode_status(frame, &status)) {
        process_status(client, &status, now_ms);
    } else {
        client->diagnostics.protocol_error_count++;
    }
}

void task_link_client_update(TaskLinkClient *client, uint32_t now_ms) {
    if (client == NULL || client->link == NULL) return;

    if (client->last_heartbeat_ms == 0U ||
        elapsed_at_least(now_ms, client->last_heartbeat_ms,
                         client->config.heartbeat_interval_ms)) {
        (void)send_heartbeat(client);
        client->last_heartbeat_ms = now_ms;
    }
    if (client->executor_session_id != 0U &&
        elapsed_at_least(now_ms, client->last_receive_ms,
                         client->config.link_timeout_ms)) {
        client->executor_session_id = 0U;
        record_peer_failure(client, TASK_FAILURE_LINK_TIMEOUT);
    }
    if (client->command_active &&
        (client->result.status == TASK_STEP_RUNNING || client->cancel_pending) &&
        elapsed_at_least(now_ms, client->last_command_send_ms,
                         client->config.command_retry_ms)) {
        if (send_command(client, client->cancel_pending ? TASK_COMMAND_CANCEL
                                                        : TASK_COMMAND_START)) {
            client->diagnostics.retry_count++;
        }
        client->last_command_send_ms = now_ms;
    }
}

void task_link_client_handle_link_error(TaskLinkClient *client) {
    if (client != NULL) record_peer_failure(client, TASK_FAILURE_LINK_TIMEOUT);
}

static bool client_start(void *context, const TaskStepCommand *command,
                         uint32_t now_ms) {
    TaskLinkClient *client = (TaskLinkClient *)context;
    if (client == NULL || command == NULL || client->command_active ||
        !task_action_is_valid(command->action) ||
        (client->config.allowed_actions_mask &
         TASK_ACTION_BIT(command->action)) == 0U) {
        return false;
    }
    if (client->executor_session_id == 0U) {
        client->result = (TaskActionResult){TASK_STEP_FAILED,
                                            TASK_FAILURE_EXECUTOR_UNAVAILABLE};
        client->command_active = true;
        client->cancel_pending = false;
        return true;
    }
    memset(&client->command, 0, sizeof(client->command));
    client->command.requester_session_id = client->requester_session_id;
    client->command.command_id = client->next_command_id++;
    if (client->next_command_id == 0U) client->next_command_id = 1U;
    client->command.type = TASK_COMMAND_START;
    client->command.step = *command;
    client->result.status = TASK_STEP_RUNNING;
    client->result.failure = TASK_FAILURE_NONE;
    client->command_active = true;
    client->cancel_pending = false;
    client->last_command_send_ms = now_ms;
    if (!send_command(client, TASK_COMMAND_START)) {
        client->command_active = false;
        return false;
    }
    return true;
}

static TaskActionResult client_update(void *context, uint32_t now_ms) {
    (void)now_ms;
    TaskLinkClient *client = (TaskLinkClient *)context;
    if (client == NULL || !client->command_active) {
        const TaskActionResult invalid = {
            TASK_STEP_FAILED, TASK_FAILURE_PROTOCOL};
        return invalid;
    }
    const TaskActionResult result = client->result;
    if (task_step_status_is_terminal(result.status) &&
        !client->cancel_pending) {
        client->command_active = false;
    }
    return result;
}

static void client_cancel(void *context, uint32_t now_ms) {
    TaskLinkClient *client = (TaskLinkClient *)context;
    if (client == NULL || !client->command_active) return;
    (void)send_command(client, TASK_COMMAND_CANCEL);
    client->last_command_send_ms = now_ms;
    client->result.status = TASK_STEP_CANCELLED;
    client->result.failure = TASK_FAILURE_NONE;
    client->cancel_pending = true;
}

TaskActionExecutor task_link_client_executor(TaskLinkClient *client) {
    const TaskActionExecutor executor = {
        .context = client,
        .start = client_start,
        .update = client_update,
        .cancel = client_cancel,
    };
    return executor;
}

bool task_link_client_take_peer_failure(TaskLinkClient *client,
                                        TaskFailure *failure_out) {
    if (client == NULL || failure_out == NULL ||
        client->peer_failure == TASK_FAILURE_NONE) {
        return false;
    }
    *failure_out = client->peer_failure;
    client->peer_failure = TASK_FAILURE_NONE;
    return true;
}
