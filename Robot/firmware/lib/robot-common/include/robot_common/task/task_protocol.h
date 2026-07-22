/** @file task_protocol.h
 *  @brief Explicit task command, status, and heartbeat wire messages.
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
    TASK_CONTROLLER_DRIVETRAIN = 0,
    TASK_CONTROLLER_ARM,
} TaskControllerId;

typedef struct {
    uint32_t coordinator_session_id;
    uint32_t command_id;
    TaskCommandType type;
    TaskStepCommand step;
} TaskCommandMessage;

typedef struct {
    uint32_t coordinator_session_id;
    uint32_t arm_session_id;
    uint32_t execution_id;
    uint32_t command_id;
    TaskStepStatus status;
    TaskFailure failure;
} TaskStatusMessage;

typedef struct {
    TaskControllerId sender;
    uint32_t session_id;
} TaskHeartbeatMessage;

bool task_protocol_encode_command(const TaskCommandMessage *message,
                                  PacketFrame *frame_out);
bool task_protocol_decode_command(const PacketFrame *frame,
                                  TaskCommandMessage *message_out);
bool task_protocol_encode_status(const TaskStatusMessage *message,
                                 PacketFrame *frame_out);
bool task_protocol_decode_status(const PacketFrame *frame,
                                 TaskStatusMessage *message_out);
bool task_protocol_encode_heartbeat(const TaskHeartbeatMessage *message,
                                    PacketFrame *frame_out);
bool task_protocol_decode_heartbeat(const PacketFrame *frame,
                                    TaskHeartbeatMessage *message_out);
bool task_protocol_sequence_is_newer(uint32_t candidate, uint32_t reference);

#ifdef __cplusplus
}
#endif
