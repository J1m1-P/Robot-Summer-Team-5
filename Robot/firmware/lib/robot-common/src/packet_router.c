/** @file packet_router.c
 *  @brief Implements message multiplexing over a shared framed UART transport.
 */
#include <robot_common/packet_router.h>

#include <stddef.h>
#include <string.h>

// Creates an empty routing table without initializing or taking ownership of the UART driver.
bool packet_router_init(PacketRouter *router, UartLink *link) {
    if (router == NULL || link == NULL) return false;
    memset(router, 0, sizeof(*router));
    router->link = link;
    return true;
}

// Installs the sole consumer for a packet type so protocols cannot steal each other's packets.
bool packet_router_set_handler(PacketRouter *router,
                               PacketMessageType message_type,
                               PacketRouterHandler handler, void *context) {
    if (router == NULL || message_type <= PACKET_TYPE_INVALID ||
        message_type >= PACKET_TYPE_MAX) {
        return false;
    }
    router->routes[message_type].handler = handler;
    router->routes[message_type].context = handler == NULL ? NULL : context;
    return true;
}

// Drains the physical link centrally, then dispatches frames by their protocol-level type.
esp_err_t packet_router_update(PacketRouter *router, uint32_t now_ms) {
    if (router == NULL || router->link == NULL) return ESP_ERR_INVALID_ARG;

    const esp_err_t update_error = uart_link_update(router->link);
    if (update_error != ESP_OK) return update_error;

    PacketFrame frame = {0};
    while (uart_link_take_packet(router->link, &frame) == ESP_OK) {
        if (frame.message_type > (uint8_t)PACKET_TYPE_INVALID &&
            frame.message_type < (uint8_t)PACKET_TYPE_MAX) {
            PacketRoute *route = &router->routes[frame.message_type];
            if (route->handler != NULL) {
                route->handler(route->context, &frame, now_ms);
                router->packets_routed++;
                continue;
            }
        }
        router->packets_unhandled++;
    }
    return ESP_OK;
}
