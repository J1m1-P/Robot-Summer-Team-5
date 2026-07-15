#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PACKET_MAX_PAYLOAD_SIZE 64U

typedef enum {
    PACKET_TYPE_INVALID = 0,
    PACKET_TYPE_ODOMETRY,
    PACKET_TYPE_COMMAND,
    PACKET_TYPE_STATUS,
    PACKET_TYPE_MAX
} PacketMessageType;

typedef struct {
    uint8_t message_type;
    uint8_t payload_len;
    uint8_t payload[PACKET_MAX_PAYLOAD_SIZE];
} PacketFrame;

#ifdef __cplusplus
}
#endif
