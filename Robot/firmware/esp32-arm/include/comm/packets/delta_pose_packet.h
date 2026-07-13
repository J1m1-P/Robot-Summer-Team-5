#pragma once

#include <stdbool.h>

#include "comm/uart_link.h"
#include "sensing/pmw3610_fusion.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DELTA_POSE_PACKET_PAYLOAD_SIZE 13U

esp_err_t delta_pose_packet_send(UartLink *link, const DeltaPose *delta, bool valid);
bool delta_pose_packet_is(const PacketFrame *packet);
esp_err_t delta_pose_packet_decode(const PacketFrame *packet, DeltaPose *delta_out,
                                   bool *valid_out);

#ifdef __cplusplus
}
#endif
