/** @file task_protocol.c
 *  @brief Implements portable little-endian task message serialization.
 */
#include <robot_common/task/task_protocol.h>

#include <stddef.h>
#include <string.h>

#define TASK_COMMAND_PAYLOAD_SIZE 36U
#define TASK_STATUS_PAYLOAD_SIZE 18U
#define TASK_HEARTBEAT_PAYLOAD_SIZE 5U

// Writes a 32-bit integer in a CPU-independent little-endian wire format.
static void encode_u32(uint8_t *output, uint32_t value) {
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8U);
    output[2] = (uint8_t)(value >> 16U);
    output[3] = (uint8_t)(value >> 24U);
}

// Reconstructs a little-endian integer without alignment-dependent pointer casts.
static uint32_t decode_u32(const uint8_t *input) {
    return (uint32_t)input[0] | ((uint32_t)input[1] << 8U) |
           ((uint32_t)input[2] << 16U) | ((uint32_t)input[3] << 24U);
}

// Preserves an IEEE-754 float's bits before using the integer encoder.
static void encode_float(uint8_t *output, float value) {
    uint32_t bits = 0U;
    memcpy(&bits, &value, sizeof(bits));
    encode_u32(output, bits);
}

// Reconstructs a float through memcpy to avoid strict-aliasing violations.
static float decode_float(const uint8_t *input) {
    const uint32_t bits = decode_u32(input);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

// Shares command validation between encoding and decoding.
static bool command_is_valid(const TaskCommandMessage *message) {
    return message != NULL && message->requester_session_id != 0U &&
           message->step.execution_id != 0U && message->command_id != 0U &&
           message->type <= TASK_COMMAND_CANCEL &&
           task_action_is_valid(message->step.action);
}

// Enforces the one valid failure-code rule for each terminal/running status.
static bool result_is_valid(TaskStepStatus status, TaskFailure failure) {
    if (status < TASK_STEP_RUNNING || status > TASK_STEP_FAILED ||
        failure < TASK_FAILURE_NONE || failure >= TASK_FAILURE_COUNT) {
        return false;
    }
    return status == TASK_STEP_FAILED ? failure != TASK_FAILURE_NONE
                                      : failure == TASK_FAILURE_NONE;
}

// Shares status identity and result validation between both wire directions.
static bool status_is_valid(const TaskStatusMessage *message) {
    return message != NULL && message->requester_session_id != 0U &&
           message->executor_session_id != 0U && message->execution_id != 0U &&
           message->command_id != 0U &&
           result_is_valid(message->status, message->failure);
}

// Rejects reserved sessions and controller IDs before or after serialization.
static bool heartbeat_is_valid(const TaskHeartbeatMessage *message) {
    return message != NULL && message->sender < TASK_ENDPOINT_COUNT &&
           message->session_id != 0U;
}

// Validates and serializes a start/cancel command into an opaque packet payload.
bool task_protocol_encode_command(const TaskCommandMessage *message,
                                  PacketFrame *frame_out) {
    if (frame_out == NULL || !command_is_valid(message)) {
        return false;
    }

    memset(frame_out, 0, sizeof(*frame_out));
    frame_out->message_type = PACKET_TYPE_TASK_COMMAND;
    frame_out->payload_len = TASK_COMMAND_PAYLOAD_SIZE;
    encode_u32(&frame_out->payload[0], message->requester_session_id);
    encode_u32(&frame_out->payload[4], message->step.execution_id);
    encode_u32(&frame_out->payload[8], message->command_id);
    frame_out->payload[12] = (uint8_t)message->type;
    frame_out->payload[13] = (uint8_t)message->step.action;
    frame_out->payload[14] = message->step.step;
    frame_out->payload[15] =
        (uint8_t)message->step.tape_following.direction;
    encode_float(&frame_out->payload[16],
                 message->step.tape_following.speed_mps);
    encode_float(&frame_out->payload[20],
                 message->step.tape_following.distance_m);
    encode_float(&frame_out->payload[24], message->step.parameters.amount);
    encode_float(&frame_out->payload[28], message->step.parameters.speed);
    encode_u32(&frame_out->payload[32], message->step.parameters.settle_ms);
    return true;
}

// Accepts only the exact command payload shape and validated task identifiers.
bool task_protocol_decode_command(const PacketFrame *frame,
                                  TaskCommandMessage *message_out) {
    if (frame == NULL || message_out == NULL ||
        frame->message_type != PACKET_TYPE_TASK_COMMAND ||
        frame->payload_len != TASK_COMMAND_PAYLOAD_SIZE) {
        return false;
    }

    TaskCommandMessage decoded = {0};
    decoded.requester_session_id = decode_u32(&frame->payload[0]);
    decoded.step.execution_id = decode_u32(&frame->payload[4]);
    decoded.command_id = decode_u32(&frame->payload[8]);
    decoded.type = (TaskCommandType)frame->payload[12];
    decoded.step.action = (TaskAction)frame->payload[13];
    decoded.step.step = frame->payload[14];
    decoded.step.tape_following.direction =
        (TapeDirection)frame->payload[15];
    decoded.step.tape_following.speed_mps =
        decode_float(&frame->payload[16]);
    decoded.step.tape_following.distance_m =
        decode_float(&frame->payload[20]);
    decoded.step.parameters.amount = decode_float(&frame->payload[24]);
    decoded.step.parameters.speed = decode_float(&frame->payload[28]);
    decoded.step.parameters.settle_ms = decode_u32(&frame->payload[32]);

    if (!command_is_valid(&decoded)) return false;
    *message_out = decoded;
    return true;
}

// Serializes an executor result while enforcing status/failure consistency.
bool task_protocol_encode_status(const TaskStatusMessage *message,
                                 PacketFrame *frame_out) {
    if (frame_out == NULL || !status_is_valid(message)) {
        return false;
    }

    memset(frame_out, 0, sizeof(*frame_out));
    frame_out->message_type = PACKET_TYPE_TASK_STATUS;
    frame_out->payload_len = TASK_STATUS_PAYLOAD_SIZE;
    encode_u32(&frame_out->payload[0], message->requester_session_id);
    encode_u32(&frame_out->payload[4], message->executor_session_id);
    encode_u32(&frame_out->payload[8], message->execution_id);
    encode_u32(&frame_out->payload[12], message->command_id);
    frame_out->payload[16] = (uint8_t)message->status;
    frame_out->payload[17] = (uint8_t)message->failure;
    return true;
}

// Decodes a status only after its size, identities, and result combination are valid.
bool task_protocol_decode_status(const PacketFrame *frame,
                                 TaskStatusMessage *message_out) {
    if (frame == NULL || message_out == NULL ||
        frame->message_type != PACKET_TYPE_TASK_STATUS ||
        frame->payload_len != TASK_STATUS_PAYLOAD_SIZE) {
        return false;
    }

    TaskStatusMessage decoded = {0};
    decoded.requester_session_id = decode_u32(&frame->payload[0]);
    decoded.executor_session_id = decode_u32(&frame->payload[4]);
    decoded.execution_id = decode_u32(&frame->payload[8]);
    decoded.command_id = decode_u32(&frame->payload[12]);
    decoded.status = (TaskStepStatus)frame->payload[16];
    decoded.failure = (TaskFailure)frame->payload[17];
    if (!status_is_valid(&decoded)) return false;
    *message_out = decoded;
    return true;
}

// Encodes the sender and boot-session identity used for reset detection.
bool task_protocol_encode_heartbeat(const TaskHeartbeatMessage *message,
                                    PacketFrame *frame_out) {
    if (frame_out == NULL || !heartbeat_is_valid(message)) {
        return false;
    }
    memset(frame_out, 0, sizeof(*frame_out));
    frame_out->message_type = PACKET_TYPE_HEARTBEAT;
    frame_out->payload_len = TASK_HEARTBEAT_PAYLOAD_SIZE;
    frame_out->payload[0] = (uint8_t)message->sender;
    encode_u32(&frame_out->payload[1], message->session_id);
    return true;
}

// Decodes a heartbeat and rejects unknown controllers or the reserved zero session.
bool task_protocol_decode_heartbeat(const PacketFrame *frame,
                                    TaskHeartbeatMessage *message_out) {
    if (frame == NULL || message_out == NULL ||
        frame->message_type != PACKET_TYPE_HEARTBEAT ||
        frame->payload_len != TASK_HEARTBEAT_PAYLOAD_SIZE) {
        return false;
    }
    TaskHeartbeatMessage decoded = {
        .sender = (TaskEndpointId)frame->payload[0],
        .session_id = decode_u32(&frame->payload[1]),
    };
    if (!heartbeat_is_valid(&decoded)) return false;
    *message_out = decoded;
    return true;
}

// Compares wrapping 32-bit command IDs as long as peers differ by less than half the range.
bool task_protocol_sequence_is_newer(uint32_t candidate, uint32_t reference) {
    return candidate != reference &&
           (int32_t)(candidate - reference) > 0;
}
