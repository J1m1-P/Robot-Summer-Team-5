/*
 * Implements the fixed wire format for command packets sent from the Pi.
 * The value byte is a signed, x100-scaled float (see uart_link.py's mirror).
 */
#include <robot_common/command_packet.h>

#include <math.h>

// Byte offsets for each field in the fixed command payload.
#define COMMAND_OPCODE_OFFSET 0U
#define COMMAND_VALUE_OFFSET 1U

// Scales CommandPacket.value into the signed byte the wire format carries.
#define COMMAND_VALUE_SCALE 100.0f
#define COMMAND_VALUE_MAX 127
#define COMMAND_VALUE_MIN (-127)

// Ensure the fixed command payload fits inside a generic packet frame.
_Static_assert(COMMAND_PACKET_PAYLOAD_SIZE <= PACKET_MAX_PAYLOAD_SIZE,
               "Command packet exceeds UART payload capacity");

// Checks that a command packet has a valid opcode and a finite value.
static bool command_packet_values_are_valid(const CommandPacket *packet) {
    return packet != NULL && packet->opcode < CMD_MAX && isfinite(packet->value);
}

// Serializes a valid command and sends it as a command frame.
esp_err_t command_packet_send(UartLink *link, const CommandPacket *packet) {
    if (link == NULL || packet == NULL) return ESP_ERR_INVALID_ARG;
    if (!command_packet_values_are_valid(packet)) return ESP_ERR_INVALID_ARG;

    long scaled = lroundf(packet->value * COMMAND_VALUE_SCALE);
    if (scaled > COMMAND_VALUE_MAX) scaled = COMMAND_VALUE_MAX;
    if (scaled < COMMAND_VALUE_MIN) scaled = COMMAND_VALUE_MIN;

    uint8_t payload[COMMAND_PACKET_PAYLOAD_SIZE] = {0};
    payload[COMMAND_OPCODE_OFFSET] = (uint8_t)packet->opcode;
    payload[COMMAND_VALUE_OFFSET] = (uint8_t)(int8_t)scaled;

    return uart_link_send(link, PACKET_TYPE_COMMAND, payload, COMMAND_PACKET_PAYLOAD_SIZE);
}

// Identifies a frame as a command by checking its type and payload length.
bool command_packet_is(const PacketFrame *frame) {
    return frame != NULL &&
           frame->message_type == (uint8_t)PACKET_TYPE_COMMAND &&
           frame->payload_len == COMMAND_PACKET_PAYLOAD_SIZE;
}

// Decodes and validates all fields from a command frame.
esp_err_t command_packet_decode(const PacketFrame *frame, CommandPacket *packet_out) {
    if (frame == NULL || packet_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!command_packet_is(frame)) return ESP_ERR_INVALID_RESPONSE;

    uint8_t opcode = frame->payload[COMMAND_OPCODE_OFFSET];
    if (opcode >= (uint8_t)CMD_MAX) return ESP_ERR_INVALID_RESPONSE;

    int8_t raw_value = (int8_t)frame->payload[COMMAND_VALUE_OFFSET];

    packet_out->opcode = (CommandOpcode)opcode;
    packet_out->value = (float)raw_value / COMMAND_VALUE_SCALE;
    return ESP_OK;
}
