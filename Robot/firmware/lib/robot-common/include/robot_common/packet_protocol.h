/*
 * Defines the common packet types and frame representation used between boards.
 * Transport-specific encoding and parsing are handled by the UART link module.
 */
#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Limits the payload stored in a single packet frame to 64 bytes.
#define PACKET_MAX_PAYLOAD_SIZE 64U

// Identifies the kind of data carried by a packet.
typedef enum {
    PACKET_TYPE_INVALID = 0,
    PACKET_TYPE_ODOMETRY,
    PACKET_TYPE_COMMAND,
    PACKET_TYPE_STATUS,
    PACKET_TYPE_PI_REQUEST,
    PACKET_TYPE_PI_REPORT,
    PACKET_TYPE_PI_READY,   // Pi -> arm, unsolicited, empty payload: camera/model ready
    PACKET_TYPE_MAX
} PacketMessageType;

// Stores one decoded packet type, payload length, and payload buffer.
typedef struct {
    uint8_t message_type;
    uint8_t payload_len;
    uint8_t payload[PACKET_MAX_PAYLOAD_SIZE];
} PacketFrame;

#ifdef __cplusplus
}
#endif
