/*
 * Defines the odometry packet shared by the arm and drivetrain firmware.
 * The API sends, identifies, and decodes its fixed wire representation.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/uart_link.h>

#ifdef __cplusplus
extern "C" {
#endif

// Stores three 32-bit floats, one 32-bit sequence number, and one validity byte.
#define ODOMETRY_PACKET_PAYLOAD_SIZE 17U

// Represents one cumulative planar odometry sample in millimeters and radians.
typedef struct {
    float x_mm;
    float y_mm;
    float theta_rad;
    uint32_t sequence;
    bool valid;
} OdometryPacket;

// Validates, encodes, and sends an odometry packet over a UART link.
esp_err_t odometry_packet_send(UartLink *link, const OdometryPacket *packet);

// Checks whether a generic packet frame has the odometry type and expected size.
bool odometry_packet_is(const PacketFrame *frame);

// Validates and decodes an odometry frame into a caller-provided structure.
esp_err_t odometry_packet_decode(const PacketFrame *frame,
                                 OdometryPacket *packet_out);

#ifdef __cplusplus
}
#endif
