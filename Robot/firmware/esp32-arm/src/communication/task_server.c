#include "communication/task_server.h"

#include <stddef.h>
#include <string.h>

static bool server_elapsed_at_least(uint32_t now, uint32_t then,
                                    uint32_t duration) {
    return (uint32_t)(now - then) >= duration;
}

static bool send_frame(UartLink *link, const PacketFrame *frame) {
    return link != NULL && frame != NULL &&
           uart_link_send(link, (PacketMessageType)frame->message_type,
                          frame->payload, frame->payload_len) == ESP_OK;
}

static bool send_heartbeat(ArmTaskServer *server) {
    const TaskHeartbeatMessage heartbeat = {
        .sender = TASK_CONTROLLER_ARM,
        .session_id = server->arm_session_id,
    };
    PacketFrame frame = {0};
    return task_protocol_encode_heartbeat(&heartbeat, &frame) &&
           send_frame(server->link, &frame);
}

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

static void reset_for_coordinator(ArmTaskServer *server,
                                  uint32_t coordinator_session_id,
                                  uint32_t now_ms) {
    if (server->coordinator_seen &&
        server->coordinator_session_id != coordinator_session_id) {
        (void)arm_manager_cancel(server->manager);
        server->previous_coordinator_session_id =
            server->coordinator_session_id;
    }
    server->coordinator_session_id = coordinator_session_id;
    server->coordinator_seen = true;
    server->has_command = false;
    server->last_receive_ms = now_ms;
}

void arm_task_server_process_command(ArmTaskServer *server,
                                     const TaskCommandMessage *command,
                                     uint32_t now_ms) {
    if (server == NULL || command == NULL) return;
    if (server->coordinator_seen &&
        command->coordinator_session_id ==
            server->previous_coordinator_session_id) {
        server->diagnostics.stale_command_count++;
        (void)send_status_for(server, command, TASK_STEP_FAILED,
                              TASK_FAILURE_STALE_MESSAGE);
        return;
    }
    if (!server->coordinator_seen ||
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
    if (!arm_manager_start(server->manager, &command->step)) {
        (void)send_status_for(server, command, TASK_STEP_FAILED,
                              TASK_FAILURE_STEP_REJECTED);
        return;
    }
    server->last_sent_status = TASK_STEP_RUNNING;
    server->last_status_ms = now_ms;
    (void)send_status_for(server, command, TASK_STEP_RUNNING,
                          TASK_FAILURE_NONE);
}

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

void arm_task_server_update(ArmTaskServer *server, uint32_t now_ms) {
    if (server == NULL || server->link == NULL || server->manager == NULL) {
        return;
    }
    arm_manager_update(server->manager);
    if (uart_link_update(server->link) != ESP_OK) {
        (void)arm_manager_cancel(server->manager);
        server->coordinator_seen = false;
        return;
    }

    PacketFrame frame = {0};
    while (uart_link_take_packet(server->link, &frame) == ESP_OK) {
        TaskHeartbeatMessage heartbeat = {0};
        TaskCommandMessage command = {0};
        if (task_protocol_decode_heartbeat(&frame, &heartbeat) &&
            heartbeat.sender == TASK_CONTROLLER_DRIVETRAIN) {
            if (server->coordinator_seen &&
                heartbeat.session_id ==
                    server->previous_coordinator_session_id) {
                continue;
            }
            if (!server->coordinator_seen ||
                server->coordinator_session_id != heartbeat.session_id) {
                reset_for_coordinator(server, heartbeat.session_id, now_ms);
            } else {
                server->last_receive_ms = now_ms;
            }
        } else if (task_protocol_decode_command(&frame, &command)) {
            arm_task_server_process_command(server, &command, now_ms);
        } else {
            server->diagnostics.protocol_error_count++;
        }
    }

    if (server->last_heartbeat_ms == 0U ||
        server_elapsed_at_least(now_ms, server->last_heartbeat_ms,
                                server->config.heartbeat_interval_ms)) {
        (void)send_heartbeat(server);
        server->last_heartbeat_ms = now_ms;
    }

    if (server->coordinator_seen &&
        server_elapsed_at_least(now_ms, server->last_receive_ms,
                                server->config.link_timeout_ms)) {
        (void)arm_manager_cancel(server->manager);
        server->coordinator_seen = false;
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

const ArmTaskServerDiagnostics *arm_task_server_get_diagnostics(
    const ArmTaskServer *server) {
    return server == NULL ? NULL : &server->diagnostics;
}
