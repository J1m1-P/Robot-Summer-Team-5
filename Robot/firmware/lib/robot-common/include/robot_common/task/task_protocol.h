/**
 * @file task_protocol.h
 * @brief Defines and serializes the task system's UART wire messages.
 *
 * Commands carry one TaskStepCommand to a remote executor, statuses return
 * correlated results, and heartbeats identify live boot sessions. The encoder
 * functions produce portable fixed-layout packet payloads.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/packet_protocol.h>
#include <robot_common/task/task.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TASK_COMMAND_START = 0,
    TASK_COMMAND_CANCEL,
} TaskCommandType;

typedef enum {
    TASK_ENDPOINT_DRIVETRAIN = 0,
    TASK_ENDPOINT_TOP,
    TASK_ENDPOINT_PI,
    TASK_ENDPOINT_COUNT,
} TaskEndpointId;

typedef struct {
    uint32_t requester_session_id; /**< Requester's current boot session. */
    uint32_t command_id; /**< Monotonic identity within that session. */
    TaskCommandType type; /**< Start or cancel operation. */
    TaskStepCommand step; /**< Action command and execution identity. */
} TaskCommandMessage;

typedef struct {
    uint32_t requester_session_id; /**< Session that issued the command. */
    uint32_t executor_session_id;  /**< Executor's current boot session. */
    uint32_t execution_id;         /**< Task run being reported. */
    uint32_t command_id;           /**< Exact command being reported. */
    TaskStepStatus status;         /**< Latest remote step state. */
    TaskFailure failure;           /**< Remote failure reason, if any. */
} TaskStatusMessage;

typedef struct {
    TaskEndpointId sender; /**< Endpoint that emitted the heartbeat. */
    uint32_t session_id;   /**< Sender's nonzero boot-session identity. */
} TaskHeartbeatMessage;

/** Encodes a validated command into a task-command PacketFrame. */
bool task_protocol_encode_command(const TaskCommandMessage *message,
                                  PacketFrame *frame_out);
/** Decodes and validates one task-command PacketFrame. */
bool task_protocol_decode_command(const PacketFrame *frame,
                                  TaskCommandMessage *message_out);
/** Encodes a correlated action result into a task-status PacketFrame. */
bool task_protocol_encode_status(const TaskStatusMessage *message,
                                 PacketFrame *frame_out);
/** Decodes and validates one task-status PacketFrame. */
bool task_protocol_decode_status(const PacketFrame *frame,
                                 TaskStatusMessage *message_out);
/** Encodes an endpoint heartbeat and boot-session identity. */
bool task_protocol_encode_heartbeat(const TaskHeartbeatMessage *message,
                                    PacketFrame *frame_out);
/** Decodes and validates one task heartbeat. */
bool task_protocol_decode_heartbeat(const PacketFrame *frame,
                                    TaskHeartbeatMessage *message_out);
/** Performs wrap-safe ordering for 32-bit command sequence numbers. */
bool task_protocol_sequence_is_newer(uint32_t candidate, uint32_t reference);

#ifdef __cplusplus
}
#endif
