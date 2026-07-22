/** @file packet_router.h
 *  @brief Routes packets from one shared UART link to message-specific consumers.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/uart_link.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*PacketRouterHandler)(void *context, const PacketFrame *frame,
                                    uint32_t now_ms);

typedef struct {
    PacketRouterHandler handler;
    void *context;
} PacketRoute;

typedef struct {
    UartLink *link;
    PacketRoute routes[PACKET_TYPE_MAX];
    uint32_t packets_routed;
    uint32_t packets_unhandled;
} PacketRouter;

// Binds one router to one physical UART link. The link remains owned by the caller.
bool packet_router_init(PacketRouter *router, UartLink *link);

// Assigns one message type to one consumer. Registering it again replaces the consumer.
bool packet_router_set_handler(PacketRouter *router, PacketMessageType message_type,
                               PacketRouterHandler handler, void *context);

// Polls the UART once and delivers every queued packet to its registered consumer.
esp_err_t packet_router_update(PacketRouter *router, uint32_t now_ms);

#ifdef __cplusplus
}
#endif
