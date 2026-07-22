#include "communication/arm_task_client.h"

#include <stddef.h>
#include <string.h>

static bool elapsed_at_least(uint32_t now, uint32_t then, uint32_t duration) {
    return (uint32_t)(now - then) >= duration;
}

// Sends a protocol-produced frame through the generic UART framing layer.
static bool send_frame(UartLink *link, const PacketFrame *frame) {
    return link != NULL && frame != NULL &&
           uart_link_send(link, (PacketMessageType)frame->message_type,
                          frame->payload, frame->payload_len) == ESP_OK;
}

// Advertises the drivetrain boot session so the arm can detect controller resets.
static bool send_heartbeat(ArmTaskClient *client) {
    const TaskHeartbeatMessage heartbeat = {
        .sender = TASK_CONTROLLER_DRIVETRAIN,
        .session_id = client->coordinator_session_id,
    };
    PacketFrame frame = {0};
    return task_protocol_encode_heartbeat(&heartbeat, &frame) &&
           send_frame(client->link, &frame);
}

// Sends either the initial arm command or an idempotent retry/cancellation.
static bool send_command(ArmTaskClient *client, TaskCommandType type) {
    TaskCommandMessage message = client->command;
    message.type = type;
    PacketFrame frame = {0};
    return task_protocol_encode_command(&message, &frame) &&
           send_frame(client->link, &frame);
}

// Records one link-level failure for both the active executor and the coordinator.
static void record_peer_failure(ArmTaskClient *client, TaskFailure failure) {
    client->peer_failure = failure;
    if (client->command_active) {
        client->result.status = TASK_STEP_FAILED;
        client->result.failure = failure;
    }
}

// Tracks the arm boot session and turns an unexpected session change into a safe failure.
static bool accept_arm_session(ArmTaskClient *client, uint32_t session_id,
                               uint32_t now_ms) {
    const bool reset = client->arm_session_id != 0U &&
                       client->arm_session_id != session_id;
    if (reset) {
        if (session_id == client->previous_arm_session_id) {
            client->diagnostics.stale_status_count++;
            return true;
        }
        client->previous_arm_session_id = client->arm_session_id;
        record_peer_failure(client, TASK_FAILURE_PEER_RESET);
        client->command_active = false;
        client->cancel_pending = false;
    }
    client->arm_session_id = session_id;
    client->last_receive_ms = now_ms;
    return reset;
}

// Applies a status only when it belongs to this boot and the outstanding arm command.
static void process_status(ArmTaskClient *client,
                           const TaskStatusMessage *status,
                           uint32_t now_ms) {
    if (client == NULL || status == NULL) return;
    if (status->coordinator_session_id != client->coordinator_session_id) {
        client->diagnostics.stale_status_count++;
        return;
    }
    if (accept_arm_session(client, status->arm_session_id, now_ms)) return;
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

// Initializes task synchronization state while sharing, but not owning, the UART link.
bool arm_task_client_init(ArmTaskClient *client, UartLink *link,
                          uint32_t coordinator_session_id,
                          const ArmTaskClientConfig *config) {
    if (client == NULL || link == NULL || config == NULL ||
        coordinator_session_id == 0U || config->heartbeat_interval_ms == 0U ||
        config->link_timeout_ms <= config->heartbeat_interval_ms ||
        config->command_retry_ms == 0U) {
        return false;
    }
    memset(client, 0, sizeof(*client));
    client->link = link;
    client->config = *config;
    client->coordinator_session_id = coordinator_session_id;
    client->next_command_id = 1U;
    client->result.status = TASK_STEP_NOT_STARTED;
    return true;
}

// Handles task packets selected by PacketRouter; unrelated link traffic never reaches this module.
void arm_task_client_process_packet(void *context, const PacketFrame *frame,
                                    uint32_t now_ms) {
    ArmTaskClient *client = (ArmTaskClient *)context;
    if (client == NULL || frame == NULL) return;

    TaskHeartbeatMessage heartbeat = {0};
    TaskStatusMessage status = {0};
    if (task_protocol_decode_heartbeat(frame, &heartbeat) &&
        heartbeat.sender == TASK_CONTROLLER_ARM) {
        (void)accept_arm_session(client, heartbeat.session_id, now_ms);
    } else if (task_protocol_decode_status(frame, &status)) {
        process_status(client, &status, now_ms);
    } else {
        client->diagnostics.protocol_error_count++;
    }
}

// Advances communication timers and retries after the composition root has polled the router.
void arm_task_client_update(ArmTaskClient *client, uint32_t now_ms) {
    if (client == NULL || client->link == NULL) return;

    if (client->last_heartbeat_ms == 0U ||
        elapsed_at_least(now_ms, client->last_heartbeat_ms,
                         client->config.heartbeat_interval_ms)) {
        (void)send_heartbeat(client);
        client->last_heartbeat_ms = now_ms;
    }

    if (client->arm_session_id != 0U &&
        elapsed_at_least(now_ms, client->last_receive_ms,
                         client->config.link_timeout_ms)) {
        client->arm_session_id = 0U;
        record_peer_failure(client, TASK_FAILURE_LINK_TIMEOUT);
    }

    if (client->command_active &&
        (client->result.status == TASK_STEP_RUNNING ||
         client->cancel_pending) &&
        elapsed_at_least(now_ms, client->last_command_send_ms,
                         client->config.command_retry_ms)) {
        if (send_command(client, client->cancel_pending
                                     ? TASK_COMMAND_CANCEL
                                     : TASK_COMMAND_START)) {
            client->diagnostics.retry_count++;
        }
        client->last_command_send_ms = now_ms;
    }
}

// Fails immediately on a UART driver error instead of waiting for the peer timeout.
void arm_task_client_handle_link_error(ArmTaskClient *client) {
    if (client == NULL) return;
    record_peer_failure(client, TASK_FAILURE_LINK_TIMEOUT);
}

// Starts one remote arm action and assigns it a nonzero, retry-stable command ID.
static bool client_start(void *context, const TaskStepCommand *command,
                         uint32_t now_ms) {
    ArmTaskClient *client = (ArmTaskClient *)context;
    if (client == NULL || command == NULL || client->command_active ||
        client->arm_session_id == 0U ||
        (command->action != TASK_ACTION_PICK_UP_BLOCK &&
         command->action != TASK_ACTION_BUILD_TOWER)) {
        return false;
    }
    memset(&client->command, 0, sizeof(client->command));
    client->command.coordinator_session_id = client->coordinator_session_id;
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

// Returns the latest remote result to the coordinator and closes terminal commands.
static TaskActionResult client_update(void *context, uint32_t now_ms) {
    (void)now_ms;
    ArmTaskClient *client = (ArmTaskClient *)context;
    if (client == NULL || !client->command_active) {
        const TaskActionResult invalid = {
            TASK_STEP_FAILED, TASK_FAILURE_PROTOCOL};
        return invalid;
    }
    const TaskActionResult result = client->result;
    if (task_step_status_is_terminal(result.status)) {
        client->command_active = false;
    }
    return result;
}

// Requests remote cancellation and keeps the command alive until terminal acknowledgement.
static void client_cancel(void *context, uint32_t now_ms) {
    ArmTaskClient *client = (ArmTaskClient *)context;
    if (client == NULL || !client->command_active) return;
    (void)send_command(client, TASK_COMMAND_CANCEL);
    client->last_command_send_ms = now_ms;
    client->result.status = TASK_STEP_CANCELLED;
    client->result.failure = TASK_FAILURE_NONE;
    client->cancel_pending = true;
}

// Adapts the UART-backed client to the coordinator's transport-independent executor interface.
TaskActionExecutor arm_task_client_executor(ArmTaskClient *client) {
    const TaskActionExecutor executor = {
        .context = client,
        .start = client_start,
        .update = client_update,
        .cancel = client_cancel,
    };
    return executor;
}

// Transfers one pending peer failure to the composition root exactly once.
bool arm_task_client_take_peer_failure(ArmTaskClient *client,
                                       TaskFailure *failure_out) {
    if (client == NULL || failure_out == NULL ||
        client->peer_failure == TASK_FAILURE_NONE) {
        return false;
    }
    *failure_out = client->peer_failure;
    client->peer_failure = TASK_FAILURE_NONE;
    return true;
}
