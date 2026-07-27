/* Implements the cache that stores the arm board's latest decoded odometry packet. */
#include "comm/odometry_link.h"

#include <stddef.h>
#include <string.h>

void odometry_link_reset(Pmw3610OdometryLink *link) {
    if (link != NULL) memset(link, 0, sizeof(*link));
}

void odometry_link_poll(Pmw3610OdometryLink *link, UartLink *uart_link) {
    if (link == NULL || uart_link == NULL) return;

    (void)uart_link_update(uart_link);
    if (!uart_link_has_packet(uart_link)) return;

    PacketFrame frame = {0};
    if (uart_link_take_packet(uart_link, &frame) != ESP_OK) return;
    odometry_link_ingest(link, &frame);
}

void odometry_link_ingest(Pmw3610OdometryLink *link, const PacketFrame *frame) {
    if (link == NULL || frame == NULL || !odometry_packet_is(frame)) return;

    OdometryPacket decoded = {0};
    if (odometry_packet_decode(frame, &decoded) != ESP_OK) {
        link->decode_failures++;
        return;
    }

    link->latest = decoded;
    link->has_packet = true;
}
