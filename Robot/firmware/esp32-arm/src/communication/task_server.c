#include "communication/task_server.h"

#include <stddef.h>
#include <string.h>

static bool server_elapsed_at_least(uint32_t now, uint32_t then,
                                    uint32_t duration) {
    return (uint32_t)(now - then) >= duration;
}

// Sends a protocol-produced frame through the generic UART framing layer.
static bool send_frame(UartLink *link, const PacketFrame *frame) {
    return link != NULL && frame != NULL &&
           uart_link_send(link, (PacketMessageType)frame->message_type,
                          frame->payload, frame->payload_len) == ESP_OK;
}

// Advertises the arm boot session so the drivetrain can detect arm resets.
static bool send_heartbeat(ArmTaskServer *server) {
    const TaskHeartbeatMessage heartbeat = {
        .sender = TASK_CONTROLLER_ARM,
        .session_id = server->arm_session_id,
    };
    PacketFrame frame = {0};
    return task_protocol_encode_heartbeat(&heartbeat, &frame) &&
           send_frame(server->link, &frame);
}

// Serializes the current arm result while echoing the command identity used for retries.
static bool send_status_for(ArmTaskServer *server,
                            const TaskCommandMessage *command,
                            TaskStepStatus status, TaskFailure failure) {
    const TaskStatusMessage message = {
        .coordinator_session_id = command->coordinator_session_id,
        .arm_session_id = server->arm_session_id,
        .execution_id = command->step.execution_id,
        .command_id = command->command_id,
        .status = status,
        .failure = failure,
    };
    PacketFrame frame = {0};
    return task_protocol_encode_status(&message, &frame) &&
           send_frame(server->link, &frame);
}

// Cancels old work and starts synchronization state for a newly observed coordinator boot.
static void reset_for_coordinator(ArmTaskServer *server,
                                  uint32_t coordinator_session_id,
                                  uint32_t now_ms) {
    if (server->coordinator_session_id != 0U &&
        server->coordinator_session_id != coordinator_session_id) {
        (void)arm_manager_cancel(server->manager);
        server->previous_coordinator_session_id =
            server->coordinator_session_id;
    }
    server->coordinator_session_id = coordinator_session_id;
    server->has_command = false;
    server->last_receive_ms = now_ms;
}

// Validates command ordering, handles idempotent retries, and delegates new work to ArmManager.
static void process_command(ArmTaskServer *server,
                            const TaskCommandMessage *command,
                            uint32_t now_ms) {
    if (server == NULL || command == NULL) return;
    if (server->coordinator_session_id != 0U &&
        command->coordinator_session_id ==
            server->previous_coordinator_session_id) {
        server->diagnostics.stale_command_count++;
        (void)send_status_for(server, command, TASK_STEP_FAILED,
                              TASK_FAILURE_STALE_MESSAGE);
        return;
    }
    if (server->coordinator_session_id == 0U ||
        server->coordinator_session_id != command->coordinator_session_id) {
        reset_for_coordinator(server, command->coordinator_session_id, now_ms);
    }
    server->last_receive_ms = now_ms;

    if (server->has_command &&
        command->step.execution_id ==
            server->command.step.execution_id &&
        command->command_id == server->command.command_id) {
        server->diagnostics.duplicate_command_count++;
        if (command->type == TASK_COMMAND_CANCEL) {
            (void)arm_manager_cancel(server->manager);
        }
        TaskFailure failure = TASK_FAILURE_NONE;
        const TaskStepStatus status =
            arm_manager_get_status(server->manager, &failure);
        (void)send_status_for(server, command, status, failure);
        return;
    }

    if (server->has_command &&
        !task_protocol_sequence_is_newer(command->command_id,
                                         server->command.command_id)) {
        server->diagnostics.stale_command_count++;
        (void)send_status_for(server, command, TASK_STEP_FAILED,
                              TASK_FAILURE_STALE_MESSAGE);
        return;
    }

    if (command->type == TASK_COMMAND_CANCEL) {
        server->diagnostics.stale_command_count++;
        (void)send_status_for(server, command, TASK_STEP_FAILED,
                              TASK_FAILURE_STALE_MESSAGE);
        return;
    }

    TaskFailure current_failure = TASK_FAILURE_NONE;
    if (arm_manager_get_status(server->manager, &current_failure) ==
        TASK_STEP_RUNNING) {
        (void)send_status_for(server, command, TASK_STEP_FAILED,
                              TASK_FAILURE_BUSY);
        return;
    }

    server->command = *command;
    server->has_command = true;
    if (!arm_manager_start(server->manager, &command->step, now_ms)) {
        (void)send_status_for(server, command, TASK_STEP_FAILED,
                              TASK_FAILURE_STEP_REJECTED);
        return;
    }
    server->last_sent_status = TASK_STEP_RUNNING;
    server->last_status_ms = now_ms;
    (void)send_status_for(server, command, TASK_STEP_RUNNING,
                          TASK_FAILURE_NONE);
}

// Initializes server synchronization state while sharing, but not owning, the UART link.
bool arm_task_server_init(ArmTaskServer *server, UartLink *link,
                          ArmManager *manager, uint32_t arm_session_id,
                          const ArmTaskServerConfig *config) {
    if (server == NULL || link == NULL || manager == NULL || config == NULL ||
        arm_session_id == 0U || config->heartbeat_interval_ms == 0U ||
        config->link_timeout_ms <= config->heartbeat_interval_ms ||
        config->status_interval_ms == 0U) {
        return false;
    }
    memset(server, 0, sizeof(*server));
    server->link = link;
    server->manager = manager;
    server->config = *config;
    server->arm_session_id = arm_session_id;
    server->last_sent_status = TASK_STEP_NOT_STARTED;
    return true;
}

// Handles task packets selected by PacketRouter; unrelated link traffic never reaches this module.
void arm_task_server_process_packet(void *context, const PacketFrame *frame,
                                    uint32_t now_ms) {
    ArmTaskServer *server = (ArmTaskServer *)context;
    if (server == NULL || frame == NULL) return;

    TaskHeartbeatMessage heartbeat = {0};
    TaskCommandMessage command = {0};
    if (task_protocol_decode_heartbeat(frame, &heartbeat) &&
        heartbeat.sender == TASK_CONTROLLER_DRIVETRAIN) {
        if (server->coordinator_session_id != 0U &&
            heartbeat.session_id == server->previous_coordinator_session_id) {
            return;
        }
        if (server->coordinator_session_id == 0U ||
            server->coordinator_session_id != heartbeat.session_id) {
            reset_for_coordinator(server, heartbeat.session_id, now_ms);
        } else {
            server->last_receive_ms = now_ms;
        }
    } else if (task_protocol_decode_command(frame, &command)) {
        process_command(server, &command, now_ms);
    } else {
        server->diagnostics.protocol_error_count++;
    }
}

// Advances arm execution and communication timers after the composition root polls the router.
void arm_task_server_update(ArmTaskServer *server, uint32_t now_ms) {
    if (server == NULL || server->link == NULL || server->manager == NULL) {
        return;
    }
    if (server->last_heartbeat_ms == 0U ||
        server_elapsed_at_least(now_ms, server->last_heartbeat_ms,
                                server->config.heartbeat_interval_ms)) {
        (void)send_heartbeat(server);
        server->last_heartbeat_ms = now_ms;
    }

    if (server->coordinator_session_id != 0U &&
        server_elapsed_at_least(now_ms, server->last_receive_ms,
                                server->config.link_timeout_ms)) {
        (void)arm_manager_cancel(server->manager);
        server->coordinator_session_id = 0U;
        server->has_command = false;
    }

    if (server->has_command) {
        TaskFailure failure = TASK_FAILURE_NONE;
        const TaskStepStatus status =
            arm_manager_get_status(server->manager, &failure);
        if (status != server->last_sent_status ||
            server_elapsed_at_least(now_ms, server->last_status_ms,
                                    server->config.status_interval_ms)) {
            (void)send_status_for(server, &server->command, status, failure);
            server->last_sent_status = status;
            server->last_status_ms = now_ms;
        }
    }
}

// Stops remote work immediately when the shared ESP32 UART driver reports an error.
void arm_task_server_handle_link_error(ArmTaskServer *server) {
    if (server == NULL || server->manager == NULL) return;
    (void)arm_manager_cancel(server->manager);
    server->coordinator_session_id = 0U;
    server->has_command = false;
}
