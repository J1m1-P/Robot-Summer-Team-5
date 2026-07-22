/** @file task_protocol.c
 *  @brief Implements portable little-endian task message serialization.
 */
#include <robot_common/task/task_protocol.h>

#include <stddef.h>
#include <string.h>

#define TASK_COMMAND_PAYLOAD_SIZE 24U
#define TASK_STATUS_PAYLOAD_SIZE 18U
#define TASK_HEARTBEAT_PAYLOAD_SIZE 5U

static void encode_u32(uint8_t *output, uint32_t value) {
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8U);
    output[2] = (uint8_t)(value >> 16U);
    output[3] = (uint8_t)(value >> 24U);
}

static uint32_t decode_u32(const uint8_t *input) {
    return (uint32_t)input[0] | ((uint32_t)input[1] << 8U) |
           ((uint32_t)input[2] << 16U) | ((uint32_t)input[3] << 24U);
}

static void encode_float(uint8_t *output, float value) {
    uint32_t bits = 0U;
    memcpy(&bits, &value, sizeof(bits));
    encode_u32(output, bits);
}

static float decode_float(const uint8_t *input) {
    const uint32_t bits = decode_u32(input);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

bool task_protocol_encode_command(const TaskCommandMessage *message,
                                  PacketFrame *frame_out) {
    if (message == NULL || frame_out == NULL ||
        message->coordinator_session_id == 0U ||
        message->step.execution_id == 0U || message->command_id == 0U ||
        message->type > TASK_COMMAND_CANCEL ||
        !task_action_is_valid(message->step.action)) {
        return false;
    }

    memset(frame_out, 0, sizeof(*frame_out));
    frame_out->message_type = PACKET_TYPE_TASK_COMMAND;
    frame_out->payload_len = TASK_COMMAND_PAYLOAD_SIZE;
    encode_u32(&frame_out->payload[0], message->coordinator_session_id);
    encode_u32(&frame_out->payload[4], message->step.execution_id);
    encode_u32(&frame_out->payload[8], message->command_id);
    frame_out->payload[12] = (uint8_t)message->type;
    frame_out->payload[13] = (uint8_t)message->step.action;
    frame_out->payload[14] = message->step.step;
    frame_out->payload[15] =
        (uint8_t)message->step.params.tape_following.direction;
    encode_float(&frame_out->payload[16],
                 message->step.params.tape_following.speed_mps);
    encode_float(&frame_out->payload[20],
                 message->step.params.tape_following.distance_m);
    return true;
}

bool task_protocol_decode_command(const PacketFrame *frame,
                                  TaskCommandMessage *message_out) {
    if (frame == NULL || message_out == NULL ||
        frame->message_type != PACKET_TYPE_TASK_COMMAND ||
        frame->payload_len != TASK_COMMAND_PAYLOAD_SIZE) {
        return false;
    }

    TaskCommandMessage decoded = {0};
    decoded.coordinator_session_id = decode_u32(&frame->payload[0]);
    decoded.step.execution_id = decode_u32(&frame->payload[4]);
    decoded.command_id = decode_u32(&frame->payload[8]);
    decoded.type = (TaskCommandType)frame->payload[12];
    decoded.step.action = (TaskAction)frame->payload[13];
    decoded.step.step = frame->payload[14];
    decoded.step.params.tape_following.direction =
        (TapeDirection)frame->payload[15];
    decoded.step.params.tape_following.speed_mps =
        decode_float(&frame->payload[16]);
    decoded.step.params.tape_following.distance_m =
        decode_float(&frame->payload[20]);

    if (decoded.coordinator_session_id == 0U ||
        decoded.step.execution_id == 0U ||
        decoded.command_id == 0U || decoded.type > TASK_COMMAND_CANCEL ||
        !task_action_is_valid(decoded.step.action)) {
        return false;
    }
    *message_out = decoded;
    return true;
}

bool task_protocol_encode_status(const TaskStatusMessage *message,
                                 PacketFrame *frame_out) {
    if (message == NULL || frame_out == NULL ||
        message->coordinator_session_id == 0U ||
        message->arm_session_id == 0U || message->execution_id == 0U ||
        message->command_id == 0U || message->status < TASK_STEP_RUNNING ||
        message->status > TASK_STEP_FAILED ||
        message->failure > TASK_FAILURE_UNAVAILABLE ||
        ((message->status == TASK_STEP_RUNNING ||
          message->status == TASK_STEP_SUCCEEDED ||
          message->status == TASK_STEP_CANCELLED) &&
         message->failure != TASK_FAILURE_NONE) ||
        (message->status == TASK_STEP_FAILED &&
         message->failure == TASK_FAILURE_NONE)) {
        return false;
    }

    memset(frame_out, 0, sizeof(*frame_out));
    frame_out->message_type = PACKET_TYPE_TASK_STATUS;
    frame_out->payload_len = TASK_STATUS_PAYLOAD_SIZE;
    encode_u32(&frame_out->payload[0], message->coordinator_session_id);
    encode_u32(&frame_out->payload[4], message->arm_session_id);
    encode_u32(&frame_out->payload[8], message->execution_id);
    encode_u32(&frame_out->payload[12], message->command_id);
    frame_out->payload[16] = (uint8_t)message->status;
    frame_out->payload[17] = (uint8_t)message->failure;
    return true;
}

bool task_protocol_decode_status(const PacketFrame *frame,
                                 TaskStatusMessage *message_out) {
    if (frame == NULL || message_out == NULL ||
        frame->message_type != PACKET_TYPE_TASK_STATUS ||
        frame->payload_len != TASK_STATUS_PAYLOAD_SIZE) {
        return false;
    }

    TaskStatusMessage decoded = {0};
    decoded.coordinator_session_id = decode_u32(&frame->payload[0]);
    decoded.arm_session_id = decode_u32(&frame->payload[4]);
    decoded.execution_id = decode_u32(&frame->payload[8]);
    decoded.command_id = decode_u32(&frame->payload[12]);
    decoded.status = (TaskStepStatus)frame->payload[16];
    decoded.failure = (TaskFailure)frame->payload[17];
    if (decoded.coordinator_session_id == 0U || decoded.arm_session_id == 0U ||
        decoded.execution_id == 0U || decoded.command_id == 0U ||
        decoded.status < TASK_STEP_RUNNING ||
        decoded.status > TASK_STEP_FAILED ||
        decoded.failure > TASK_FAILURE_UNAVAILABLE ||
        ((decoded.status == TASK_STEP_RUNNING ||
          decoded.status == TASK_STEP_SUCCEEDED ||
          decoded.status == TASK_STEP_CANCELLED) &&
         decoded.failure != TASK_FAILURE_NONE) ||
        (decoded.status == TASK_STEP_FAILED &&
         decoded.failure == TASK_FAILURE_NONE)) {
        return false;
    }
    *message_out = decoded;
    return true;
}

bool task_protocol_encode_heartbeat(const TaskHeartbeatMessage *message,
                                    PacketFrame *frame_out) {
    if (message == NULL || frame_out == NULL ||
        message->sender > TASK_CONTROLLER_ARM || message->session_id == 0U) {
        return false;
    }
    memset(frame_out, 0, sizeof(*frame_out));
    frame_out->message_type = PACKET_TYPE_HEARTBEAT;
    frame_out->payload_len = TASK_HEARTBEAT_PAYLOAD_SIZE;
    frame_out->payload[0] = (uint8_t)message->sender;
    encode_u32(&frame_out->payload[1], message->session_id);
    return true;
}

bool task_protocol_decode_heartbeat(const PacketFrame *frame,
                                    TaskHeartbeatMessage *message_out) {
    if (frame == NULL || message_out == NULL ||
        frame->message_type != PACKET_TYPE_HEARTBEAT ||
        frame->payload_len != TASK_HEARTBEAT_PAYLOAD_SIZE) {
        return false;
    }
    TaskHeartbeatMessage decoded = {
        .sender = (TaskControllerId)frame->payload[0],
        .session_id = decode_u32(&frame->payload[1]),
    };
    if (decoded.sender > TASK_CONTROLLER_ARM || decoded.session_id == 0U) {
        return false;
    }
    *message_out = decoded;
    return true;
}

bool task_protocol_sequence_is_newer(uint32_t candidate, uint32_t reference) {
    return candidate != reference &&
           (int32_t)(candidate - reference) > 0;
}
