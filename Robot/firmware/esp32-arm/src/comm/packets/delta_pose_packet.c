#include "comm/packets/delta_pose_packet.h"

#include <string.h>

#define DELTA_POSE_DX_OFFSET 0U
#define DELTA_POSE_DY_OFFSET 4U
#define DELTA_POSE_DTHETA_OFFSET 8U
#define DELTA_POSE_VALID_OFFSET 12U

_Static_assert(sizeof(float) == 4U, "DeltaPose packets require 32-bit floats");
_Static_assert(DELTA_POSE_PACKET_PAYLOAD_SIZE <= PACKET_MAX_PAYLOAD_SIZE,
               "DeltaPose packet exceeds UART payload capacity");

esp_err_t delta_pose_packet_send(UartLink *link, const DeltaPose *delta, bool valid) {
    if (link == NULL || delta == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t payload[DELTA_POSE_PACKET_PAYLOAD_SIZE];
    memcpy(&payload[DELTA_POSE_DX_OFFSET], &delta->dx_mm, sizeof(delta->dx_mm));
    memcpy(&payload[DELTA_POSE_DY_OFFSET], &delta->dy_mm, sizeof(delta->dy_mm));
    memcpy(&payload[DELTA_POSE_DTHETA_OFFSET], &delta->dtheta_rad,
           sizeof(delta->dtheta_rad));
    payload[DELTA_POSE_VALID_OFFSET] = valid ? 1U : 0U;

    return uart_link_send(link, PACKET_TYPE_ODOMETRY, payload, sizeof(payload));
}

bool delta_pose_packet_is(const PacketFrame *packet) {
    return packet != NULL && packet->message_type == (uint8_t)PACKET_TYPE_ODOMETRY &&
           packet->payload_len == DELTA_POSE_PACKET_PAYLOAD_SIZE;
}

esp_err_t delta_pose_packet_decode(const PacketFrame *packet, DeltaPose *delta_out,
                                   bool *valid_out) {
    if (packet == NULL || delta_out == NULL || valid_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!delta_pose_packet_is(packet)) return ESP_ERR_INVALID_RESPONSE;

    memcpy(&delta_out->dx_mm, &packet->payload[DELTA_POSE_DX_OFFSET],
           sizeof(delta_out->dx_mm));
    memcpy(&delta_out->dy_mm, &packet->payload[DELTA_POSE_DY_OFFSET],
           sizeof(delta_out->dy_mm));
    memcpy(&delta_out->dtheta_rad, &packet->payload[DELTA_POSE_DTHETA_OFFSET],
           sizeof(delta_out->dtheta_rad));
    *valid_out = packet->payload[DELTA_POSE_VALID_OFFSET] != 0U;
    return ESP_OK;
}
