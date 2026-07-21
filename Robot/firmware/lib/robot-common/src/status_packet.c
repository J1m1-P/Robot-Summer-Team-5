/*
 * Implements the fixed wire format for status packets sent from the ESP.
 */
#include <robot_common/status_packet.h>

// Byte offsets for each field in the fixed status payload.
#define STATUS_CODE_OFFSET 0U
#define STATUS_DETAIL_OFFSET 1U

// Ensure the fixed status payload fits inside a generic packet frame.
_Static_assert(STATUS_PACKET_PAYLOAD_SIZE <= PACKET_MAX_PAYLOAD_SIZE,
               "Status packet exceeds UART payload capacity");

// Serializes a valid status and sends it as a status frame.
esp_err_t status_packet_send(UartLink *link, const StatusPacket *packet) {
    if (link == NULL || packet == NULL) return ESP_ERR_INVALID_ARG;
    if (packet->code >= STATUS_MAX) return ESP_ERR_INVALID_ARG;

    uint8_t payload[STATUS_PACKET_PAYLOAD_SIZE] = {0};
    payload[STATUS_CODE_OFFSET] = (uint8_t)packet->code;
    payload[STATUS_DETAIL_OFFSET] = packet->detail;

    return uart_link_send(link, PACKET_TYPE_STATUS, payload, STATUS_PACKET_PAYLOAD_SIZE);
}

// Identifies a frame as a status by checking its type and payload length.
bool status_packet_is(const PacketFrame *frame) {
    return frame != NULL &&
           frame->message_type == (uint8_t)PACKET_TYPE_STATUS &&
           frame->payload_len == STATUS_PACKET_PAYLOAD_SIZE;
}

// Decodes and validates all fields from a status frame.
esp_err_t status_packet_decode(const PacketFrame *frame, StatusPacket *packet_out) {
    if (frame == NULL || packet_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!status_packet_is(frame)) return ESP_ERR_INVALID_RESPONSE;

    uint8_t code = frame->payload[STATUS_CODE_OFFSET];
    if (code >= (uint8_t)STATUS_MAX) return ESP_ERR_INVALID_RESPONSE;

    packet_out->code = (StatusCode)code;
    packet_out->detail = frame->payload[STATUS_DETAIL_OFFSET];
    return ESP_OK;
}
