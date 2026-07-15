#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/uart_link.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ODOMETRY_PACKET_PAYLOAD_SIZE 17U

typedef struct {
    float x_mm;
    float y_mm;
    float theta_rad;
    uint32_t sequence;
    bool valid;
} OdometryPacket;

esp_err_t odometry_packet_send(UartLink *link, const OdometryPacket *packet);
bool odometry_packet_is(const PacketFrame *frame);
esp_err_t odometry_packet_decode(const PacketFrame *frame,
                                 OdometryPacket *packet_out);

#ifdef __cplusplus
}
#endif
