/* Declares the cache that stores the arm board's latest decoded odometry packet. */
#pragma once

#include <stdbool.h>

#include <robot_common/odometry_packet.h>
#include <robot_common/uart_link.h>

#ifdef __cplusplus
extern "C" {
#endif

// Holds the most recently decoded PMW3610 odometry packet from the arm
// board. Owned by whoever calls odometry_link_poll() each cycle; read from
// wherever the fused odometry source needs it.
typedef struct {
    OdometryPacket latest;
    bool has_packet;
    // Counts frames that arrived with the odometry packet type (and so
    // reached odometry_packet_decode()) but were then rejected by it --
    // e.g. wrong payload length or a non-finite pose value.
    uint32_t decode_failures;
} Pmw3610OdometryLink;

// Clears the cached packet.
void odometry_link_reset(Pmw3610OdometryLink *link);

// Advances `uart_link`'s parser and, if a complete packet is now waiting,
// consumes it. Frames whose type isn't PACKET_TYPE_ODOMETRY are ignored
// (there is currently nothing else on this link to dispatch them to).
// Malformed odometry frames are dropped silently, leaving the previously
// cached packet in place. Call once per loop iteration.
//
// Only safe when nothing else also reads `uart_link` -- if arm_uart is
// shared with other traffic (e.g. robot_sequence_controller's status/
// pi-report frames), use odometry_link_ingest() from a single shared
// dispatch point instead (see robot_sequence_controller_update()).
void odometry_link_poll(Pmw3610OdometryLink *link, UartLink *uart_link);

// Decodes and caches `frame` if it's an odometry frame; otherwise a no-op.
// Malformed odometry frames are dropped silently, leaving the previously
// cached packet in place. Use this to consume a frame someone else already
// dequeued from a shared uart_link.
void odometry_link_ingest(Pmw3610OdometryLink *link, const PacketFrame *frame);

#ifdef __cplusplus
}
#endif
