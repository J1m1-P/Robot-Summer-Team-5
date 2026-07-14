#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

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

typedef enum {
    PACKET_PARSE_MAGIC_0 = 0,
    PACKET_PARSE_MAGIC_1,
    PACKET_PARSE_VERSION,
    PACKET_PARSE_TYPE,
    PACKET_PARSE_LEN,
    PACKET_PARSE_PAYLOAD,
    PACKET_PARSE_CHECKSUM
} PacketParserState;

typedef struct {
    PacketParserState state;
    uint8_t version;
    uint8_t message_type;
    uint8_t payload_len;
    uint8_t payload_index;
    uint8_t checksum_accumulator;
    uint8_t payload[PACKET_MAX_PAYLOAD_SIZE];
} PacketParser;

typedef struct {
    uart_port_t uart_num;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    uint32_t baud_rate;
    size_t rx_buffer_size;
    size_t tx_buffer_size;
} UartLinkConfig;

typedef struct {
    const UartLinkConfig *config;
    PacketParser parser;
    bool initialized;
    uint32_t packets_sent;
    uint32_t packets_received;
    uint32_t packets_overwritten;
    uint32_t checksum_errors;
    uint32_t parse_errors;
    bool has_new_packet;
    PacketFrame latest_packet;
} UartLink;

// The link must be zero-initialized before its first call to uart_link_init().
esp_err_t uart_link_init(UartLink *link, const UartLinkConfig *config);
esp_err_t uart_link_deinit(UartLink *link);

esp_err_t uart_link_update(UartLink *link);
esp_err_t uart_link_send(UartLink *link, PacketMessageType message_type, const uint8_t *payload,
                         uint8_t payload_len);
esp_err_t uart_link_take_packet(UartLink *link, PacketFrame *packet_out);

bool uart_link_has_packet(const UartLink *link);

#ifdef __cplusplus
}
#endif
