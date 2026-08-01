/* Implements the compact, fixed-size Raspberry Pi request/report format. */
#include <robot_common/pi_action_packet.h>

#include <math.h>

#define VALUE_SCALE 100.0f
#define ERROR_SCALE 1000.0f

static int8_t encode_parameter(float value) {
    long scaled = lroundf(value * VALUE_SCALE);
    if (scaled > 127L) scaled = 127L;
    if (scaled < -127L) scaled = -127L;
    return (int8_t)scaled;
}

static int16_t encode_error(float value) {
    if (value > 1.0f) value = 1.0f;
    if (value < -1.0f) value = -1.0f;
    return (int16_t)lroundf(value * ERROR_SCALE);
}

esp_err_t pi_request_packet_send(
    UartLink *link,
    const PiRequestPacket *packet) {
    if (link == NULL || packet == NULL ||
        packet->action >= PI_ACTION_MAX || !isfinite(packet->parameter)) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint8_t payload[PI_REQUEST_PACKET_PAYLOAD_SIZE] = {
        packet->request_id,
        (uint8_t)packet->action,
        (uint8_t)encode_parameter(packet->parameter),
    };
    return uart_link_send(
        link, PACKET_TYPE_PI_REQUEST, payload, sizeof(payload));
}

bool pi_request_packet_is(const PacketFrame *frame) {
    return frame != NULL &&
        frame->message_type == (uint8_t)PACKET_TYPE_PI_REQUEST &&
        frame->payload_len == PI_REQUEST_PACKET_PAYLOAD_SIZE;
}

esp_err_t pi_request_packet_decode(
    const PacketFrame *frame,
    PiRequestPacket *packet_out) {
    if (frame == NULL || packet_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!pi_request_packet_is(frame) ||
        frame->payload[1] >= (uint8_t)PI_ACTION_MAX) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    packet_out->request_id = frame->payload[0];
    packet_out->action = (PiAction)frame->payload[1];
    packet_out->parameter = (float)(int8_t)frame->payload[2] / VALUE_SCALE;
    return ESP_OK;
}

esp_err_t pi_report_packet_send(
    UartLink *link,
    const PiReportPacket *packet) {
    if (link == NULL || packet == NULL ||
        packet->action >= PI_ACTION_MAX ||
        packet->result >= PI_RESULT_MAX ||
        packet->confidence_percent > 100U ||
        !isfinite(packet->horizontal_error)) {
        return ESP_ERR_INVALID_ARG;
    }

    const int16_t error = encode_error(packet->horizontal_error);
    const uint8_t payload[PI_REPORT_PACKET_PAYLOAD_SIZE] = {
        packet->request_id,
        (uint8_t)packet->action,
        (uint8_t)packet->result,
        packet->target_id,
        (uint8_t)(error & 0xFF),
        (uint8_t)(((uint16_t)error >> 8U) & 0xFFU),
        packet->confidence_percent,
    };
    return uart_link_send(
        link, PACKET_TYPE_PI_REPORT, payload, sizeof(payload));
}

bool pi_report_packet_is(const PacketFrame *frame) {
    return frame != NULL &&
        frame->message_type == (uint8_t)PACKET_TYPE_PI_REPORT &&
        frame->payload_len == PI_REPORT_PACKET_PAYLOAD_SIZE;
}

esp_err_t pi_report_packet_decode(
    const PacketFrame *frame,
    PiReportPacket *packet_out) {
    if (frame == NULL || packet_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!pi_report_packet_is(frame) ||
        frame->payload[1] >= (uint8_t)PI_ACTION_MAX ||
        frame->payload[2] >= (uint8_t)PI_RESULT_MAX ||
        frame->payload[6] > 100U) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint16_t raw_error =
        (uint16_t)frame->payload[4] |
        ((uint16_t)frame->payload[5] << 8U);
    const int16_t error = (int16_t)raw_error;

    packet_out->request_id = frame->payload[0];
    packet_out->action = (PiAction)frame->payload[1];
    packet_out->result = (PiResultCode)frame->payload[2];
    packet_out->target_id = frame->payload[3];
    packet_out->horizontal_error = (float)error / ERROR_SCALE;
    packet_out->confidence_percent = frame->payload[6];
    return ESP_OK;
}

bool pi_ready_packet_is(const PacketFrame *frame) {
    return frame != NULL &&
        frame->message_type == (uint8_t)PACKET_TYPE_PI_READY &&
        frame->payload_len == 0U;
}
