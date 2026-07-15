#include <robot_common/odometry_packet.h>

#include <math.h>
#include <string.h>

#define ODOMETRY_X_OFFSET 0U
#define ODOMETRY_Y_OFFSET 4U
#define ODOMETRY_THETA_OFFSET 8U
#define ODOMETRY_SEQUENCE_OFFSET 12U
#define ODOMETRY_VALID_OFFSET 16U

_Static_assert(sizeof(float) == 4U, "Odometry packets require 32-bit floats");
_Static_assert(ODOMETRY_PACKET_PAYLOAD_SIZE <= PACKET_MAX_PAYLOAD_SIZE,
               "Odometry packet exceeds UART payload capacity");

static void encode_u32_le(uint8_t *destination, uint32_t value) {
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((value >> 16U) & 0xFFU);
    destination[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint32_t decode_u32_le(const uint8_t *source) {
    return (uint32_t)source[0] |
           ((uint32_t)source[1] << 8U) |
           ((uint32_t)source[2] << 16U) |
           ((uint32_t)source[3] << 24U);
}

static void encode_float_le(uint8_t *destination, float value) {
    uint32_t bits = 0U;
    memcpy(&bits, &value, sizeof(bits));
    encode_u32_le(destination, bits);
}

static float decode_float_le(const uint8_t *source) {
    uint32_t bits = decode_u32_le(source);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static bool odometry_packet_values_are_valid(const OdometryPacket *packet) {
    return packet != NULL && isfinite(packet->x_mm) && isfinite(packet->y_mm) &&
           isfinite(packet->theta_rad);
}

esp_err_t odometry_packet_send(UartLink *link, const OdometryPacket *packet) {
    if (link == NULL || packet == NULL) return ESP_ERR_INVALID_ARG;
    if (!odometry_packet_values_are_valid(packet)) return ESP_ERR_INVALID_ARG;

    uint8_t payload[ODOMETRY_PACKET_PAYLOAD_SIZE] = {0};
    encode_float_le(&payload[ODOMETRY_X_OFFSET], packet->x_mm);
    encode_float_le(&payload[ODOMETRY_Y_OFFSET], packet->y_mm);
    encode_float_le(&payload[ODOMETRY_THETA_OFFSET], packet->theta_rad);
    encode_u32_le(&payload[ODOMETRY_SEQUENCE_OFFSET], packet->sequence);
    payload[ODOMETRY_VALID_OFFSET] = packet->valid ? 1U : 0U;

    return uart_link_send(link, PACKET_TYPE_ODOMETRY, payload,
                          ODOMETRY_PACKET_PAYLOAD_SIZE);
}

bool odometry_packet_is(const PacketFrame *frame) {
    return frame != NULL &&
           frame->message_type == (uint8_t)PACKET_TYPE_ODOMETRY &&
           frame->payload_len == ODOMETRY_PACKET_PAYLOAD_SIZE;
}

esp_err_t odometry_packet_decode(const PacketFrame *frame,
                                 OdometryPacket *packet_out) {
    if (frame == NULL || packet_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!odometry_packet_is(frame)) return ESP_ERR_INVALID_RESPONSE;
    if (frame->payload[ODOMETRY_VALID_OFFSET] > 1U) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    OdometryPacket decoded = {
        .x_mm = decode_float_le(&frame->payload[ODOMETRY_X_OFFSET]),
        .y_mm = decode_float_le(&frame->payload[ODOMETRY_Y_OFFSET]),
        .theta_rad = decode_float_le(&frame->payload[ODOMETRY_THETA_OFFSET]),
        .sequence = decode_u32_le(&frame->payload[ODOMETRY_SEQUENCE_OFFSET]),
        .valid = frame->payload[ODOMETRY_VALID_OFFSET] == 1U,
    };

    if (!odometry_packet_values_are_valid(&decoded)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    *packet_out = decoded;
    return ESP_OK;
}
