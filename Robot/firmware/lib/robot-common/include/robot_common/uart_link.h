/*
 * Public interface for sending and receiving framed packets over an ESP32 UART.
 * The module owns the parser state and basic link diagnostics.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include <robot_common/packet_protocol.h>

#define UART_LINK_PACKET_QUEUE_SIZE 8U

#ifdef __cplusplus
extern "C" {
#endif

// Identifies the byte currently expected by the packet parser.
typedef enum {
    PACKET_PARSE_MAGIC_0 = 0,
    PACKET_PARSE_MAGIC_1,
    PACKET_PARSE_VERSION,
    PACKET_PARSE_TYPE,
    PACKET_PARSE_LEN,
    PACKET_PARSE_PAYLOAD,
    PACKET_PARSE_CHECKSUM
} PacketParserState;

// Stores an in-progress packet while bytes arrive from the UART.
typedef struct {
    PacketParserState state;
    uint8_t version;
    uint8_t message_type;
    uint8_t payload_len;
    uint8_t payload_index;
    uint8_t checksum_accumulator;
    uint8_t payload[PACKET_MAX_PAYLOAD_SIZE];
} PacketParser;

// Defines the UART peripheral, pins, speed, and driver buffer sizes for a link.
typedef struct {
    uart_port_t uart_num;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    uint32_t baud_rate;
    size_t rx_buffer_size;
    size_t tx_buffer_size;
} UartLinkConfig;

// Holds the runtime state, latest packet, and diagnostic counters for a UART link.
typedef struct {
    const UartLinkConfig *config;
    PacketParser parser;
    bool initialized;
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t packets_dropped;
    uint32_t checksum_errors;
    uint32_t parse_errors;
    PacketFrame packet_queue[UART_LINK_PACKET_QUEUE_SIZE];
    uint8_t packet_queue_head;
    uint8_t packet_queue_tail;
    uint8_t packet_queue_count;
} UartLink;

// Installs and configures the UART driver. The link must be zero-initialized before its first call.
// Example: UartLink link = {0};
esp_err_t uart_link_init(UartLink *link, const UartLinkConfig *config);

// Removes the UART driver and clears the link's runtime state.
esp_err_t uart_link_deinit(UartLink *link);

// Reads available UART bytes and advances the packet parser without blocking.
esp_err_t uart_link_update(UartLink *link);

// Encodes and writes one framed packet to the configured UART.
esp_err_t uart_link_send(UartLink *link, PacketMessageType message_type, const uint8_t *payload,
                         uint8_t payload_len);

// Copies out the latest complete packet and marks it as consumed.
esp_err_t uart_link_take_packet(UartLink *link, PacketFrame *packet_out);

// Reports whether a complete packet is waiting to be consumed.
bool uart_link_has_packet(const UartLink *link);

#ifdef __cplusplus
}
#endif
