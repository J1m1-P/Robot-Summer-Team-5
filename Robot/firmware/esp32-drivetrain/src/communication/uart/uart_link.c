#include "communication/uart/uart_link.h"

#include <string.h>
#include <limits.h>

#include "debug/app_log.h"

// Protocol constants
#define PACKET_MAGIC_0              0xAAU
#define PACKET_MAGIC_1              0x55U
#define PACKET_VERSION              0x01U
#define PACKET_FRAME_OVERHEAD       6U
#define PACKET_MAX_FRAME_SIZE (PACKET_MAX_PAYLOAD_SIZE + PACKET_FRAME_OVERHEAD)
#define UART_LINK_READ_CHUNK_SIZE 256U
#define UART_LINK_MAX_READ_BATCHES 4U

_Static_assert(
    PACKET_MAX_PAYLOAD_SIZE <= UINT8_MAX,
    "PACKET_MAX_PAYLOAD_SIZE must fit in uint8_t"
);

// Helper Functions
static uint8_t checksum_update(uint8_t checksum, uint8_t byte) {
    return checksum ^ byte;
}

static uint8_t calculate_checksum(uint8_t version, uint8_t message_type, uint8_t payload_len, const uint8_t *payload) {
    uint8_t checksum = 0U;

    checksum = checksum_update(checksum, version);
    checksum = checksum_update(checksum, message_type);
    checksum = checksum_update(checksum, payload_len);

    for (size_t i = 0; i < payload_len; i++) {
        checksum = checksum_update(checksum, payload[i]);
    }

    return checksum;
}

static void parser_reset(PacketParser *parser) {
    if (parser == NULL) return;

    memset(parser, 0, sizeof(*parser));
    parser->state = PACKET_PARSE_MAGIC_0;
}

static void parser_finish_packet(UartLink *link) {
    PacketParser *parser = &link->parser;
    PacketFrame *frame = &link->latest_packet;

    memset(frame, 0, sizeof(*frame));

    frame->message_type = parser->message_type;
    frame->payload_len = parser->payload_len;
    if (parser->payload_len > 0U) {
        memcpy(
            frame->payload, 
            parser->payload, 
            parser->payload_len
        );
    }

    if (link->has_new_packet) {
        link->packets_overwritten++;
    }

    link->has_new_packet = true;
    link->packets_received++;
}

static void parser_process_byte(UartLink *link, uint8_t byte) {
    PacketParser *parser = &link->parser;

    switch(parser->state) {
        case PACKET_PARSE_MAGIC_0: 
            if (byte == PACKET_MAGIC_0) {
                parser->state = PACKET_PARSE_MAGIC_1;
            }
            break;

        case PACKET_PARSE_MAGIC_1: 
            if (byte == PACKET_MAGIC_1) {
                parser->state = PACKET_PARSE_VERSION;
            }
            else if (byte == PACKET_MAGIC_0) {
                parser->state = PACKET_PARSE_MAGIC_1;
            }
            else {
                parser_reset(parser);
            }
            break;

        case PACKET_PARSE_VERSION: 
            if (byte != PACKET_VERSION) {
                link->parse_errors++;
                parser_reset(parser);
                break;
            }

            parser->version = byte;
            parser->checksum_accumulator = checksum_update(0U, byte);
            parser->state = PACKET_PARSE_TYPE;
            break;

        case PACKET_PARSE_TYPE: 
            if (byte <= (uint8_t)PACKET_TYPE_INVALID || byte >= (uint8_t)PACKET_TYPE_MAX) {
                link->parse_errors++;
                parser_reset(parser);
                break;
            }

            parser->message_type = byte;
            parser->checksum_accumulator = checksum_update(parser->checksum_accumulator, byte);
            parser->state = PACKET_PARSE_LEN; 
            break;

        case PACKET_PARSE_LEN: 
            if (byte > PACKET_MAX_PAYLOAD_SIZE) {
                link->parse_errors++;
                parser_reset(parser);
                break;
            }

            parser->payload_len = byte;
            parser->payload_index = 0U;
            parser->checksum_accumulator = checksum_update(parser->checksum_accumulator, byte);

            if (parser->payload_len == 0U) {
                parser->state = PACKET_PARSE_CHECKSUM;
            } else {
                parser->state = PACKET_PARSE_PAYLOAD;
            }
            break;

        case PACKET_PARSE_PAYLOAD: 

            if (parser->payload_index >= parser->payload_len ||
                parser->payload_index >= PACKET_MAX_PAYLOAD_SIZE) {

                link->parse_errors++;
                parser_reset(parser);
                break;
            }

            parser->payload[parser->payload_index++] = byte;
            parser->checksum_accumulator = checksum_update(parser->checksum_accumulator, byte);
            if (parser->payload_index >= parser->payload_len) {
                parser->state = PACKET_PARSE_CHECKSUM;
            }
            break;

        case PACKET_PARSE_CHECKSUM: 
            if (byte == parser->checksum_accumulator) {
                parser_finish_packet(link);
            } else {
                link->checksum_errors++;
            }

            parser_reset(parser);
            break;

        default:
            link->parse_errors++;
            parser_reset(parser);
            break;
    }
}

// Lifecyle
esp_err_t uart_link_init(UartLink *link, const UartLinkConfig *config) {
    if (link == NULL || config == NULL) return ESP_ERR_INVALID_ARG;
    if (config->baud_rate == 0U) return ESP_ERR_INVALID_ARG;
    if (config->rx_buffer_size > INT_MAX || config->tx_buffer_size > INT_MAX) return ESP_ERR_INVALID_ARG;
    if (link->initialized) return ESP_ERR_INVALID_STATE;

    memset(link, 0, sizeof(*link));

    /*
        The ESP-IDF UART driver requires:
        
        RX buffer: greater than the UART hardware FIFO size
        TX buffer: either zero or greater than the hardware FIFO size
    */

    esp_err_t err = uart_driver_install(
        config->uart_num, 
        (int)config->rx_buffer_size, 
        (int)config->tx_buffer_size, 
        0, 
        NULL,
        0
    );

    if (err != ESP_OK) {
        APP_LOGE(LOG_TAG_UART, "Failed to install uart driver: %s", esp_err_to_name(err));
        return err;
    }

    // Configure the UART using individual setter functions.
    err = uart_set_baudrate(config->uart_num, config->baud_rate);
    if (err != ESP_OK) goto cleanup;

    err = uart_set_word_length(config->uart_num, UART_DATA_8_BITS);
    if (err != ESP_OK) goto cleanup;

    err = uart_set_parity(config->uart_num, UART_PARITY_DISABLE);
    if (err != ESP_OK) goto cleanup;

    err = uart_set_stop_bits(config->uart_num, UART_STOP_BITS_1);
    if (err != ESP_OK) goto cleanup;

    err = uart_set_hw_flow_ctrl(config->uart_num, UART_HW_FLOWCTRL_DISABLE, 0U);
    if (err != ESP_OK) goto cleanup;

    err = uart_set_pin(
        config->uart_num,
        config->tx_pin,
        config->rx_pin,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    );
    if (err != ESP_OK) goto cleanup;

    link->config = config;
    parser_reset(&link->parser);

    link->initialized = true;

    return ESP_OK;

    cleanup: 
        uart_driver_delete(config->uart_num);
        memset(link, 0, sizeof(*link));
        APP_LOGE(LOG_TAG_UART, "Failed to configure the UART driver: %s", esp_err_to_name(err));
        return err; 
}

esp_err_t uart_link_deinit(UartLink *link) {
    if (link == NULL) return ESP_ERR_INVALID_ARG;
    if (!link->initialized || link->config == NULL) return ESP_ERR_INVALID_STATE;

    esp_err_t err = uart_driver_delete(link->config->uart_num);
    if (err != ESP_OK) return err;

    memset(link, 0, sizeof(*link));

    return ESP_OK;
}

// Actions
esp_err_t uart_link_send(UartLink *link, PacketMessageType message_type, const uint8_t *payload, uint8_t payload_len) {
    if (link == NULL) return ESP_ERR_INVALID_ARG;
    if (!link->initialized || link->config == NULL) return ESP_ERR_INVALID_STATE;
    if (message_type <= PACKET_TYPE_INVALID || message_type >= PACKET_TYPE_MAX) return ESP_ERR_INVALID_ARG;
    if (payload_len > PACKET_MAX_PAYLOAD_SIZE) return ESP_ERR_INVALID_SIZE;
    if (payload_len > 0U && payload == NULL) return ESP_ERR_INVALID_ARG;

    uint8_t frame[PACKET_MAX_FRAME_SIZE];
    size_t frame_index = 0U;

    const uint8_t raw_message_type = (uint8_t)message_type;

    frame[frame_index++] = PACKET_MAGIC_0;
    frame[frame_index++] = PACKET_MAGIC_1;
    frame[frame_index++] = PACKET_VERSION;
    frame[frame_index++] = raw_message_type;
    frame[frame_index++] = payload_len;

    if (payload_len > 0U) {
        memcpy(&frame[frame_index], payload, payload_len);
        frame_index += payload_len;
    }

    frame[frame_index++] = calculate_checksum(PACKET_VERSION, raw_message_type, payload_len, payload);

    int bytes_written = uart_write_bytes(link->config->uart_num, frame, frame_index);

    if (bytes_written < 0) return ESP_FAIL;
    if ((size_t)bytes_written != frame_index) return ESP_ERR_INVALID_SIZE;

    link->packets_sent++;

    return ESP_OK;
}

esp_err_t uart_link_take_packet(UartLink *link, PacketFrame *packet_out) {
    if (link == NULL || packet_out == NULL) return ESP_ERR_INVALID_ARG;
    if (!link->initialized || link->config == NULL) return ESP_ERR_INVALID_STATE;
    if (!link->has_new_packet) return ESP_ERR_NOT_FOUND;

    *packet_out = link->latest_packet;
    link->has_new_packet = false;

    return ESP_OK;
}

esp_err_t uart_link_update(UartLink *link) {
    if (link == NULL) return ESP_ERR_INVALID_ARG;
    if (!link->initialized || link->config == NULL) return ESP_ERR_INVALID_STATE;

    uint8_t read_buffer[UART_LINK_READ_CHUNK_SIZE];

    for (size_t batch = 0U; batch < UART_LINK_MAX_READ_BATCHES; batch++) { 
        int bytes_read = uart_read_bytes(link->config->uart_num, read_buffer, sizeof(read_buffer), 0);

        if (bytes_read < 0) return ESP_FAIL;

        for (int i = 0; i < bytes_read; i++) {
            parser_process_byte(link, read_buffer[i]);
        }

        if ((size_t)bytes_read < sizeof(read_buffer)) {
            break;
        }
    }

    return ESP_OK;
}

// Status
bool uart_link_has_packet(const UartLink *link) {
    if (link == NULL || !link->initialized) {
        return false;
    }

    return link->has_new_packet;
}
