#include "config/pin_map.h"
#include "config/packet_link_config.h"

const PacketLinkConfig TOP_ESP_PACKET_LINK_CONFIG = {
    .uart_num = UART_NUM_1,

    .tx_pin = PIN_TOP_ESP32_UART_TX,
    .rx_pin = PIN_TOP_ESP32_UART_RX,

    .baud_rate = 115200U,

    .rx_buffer_size = 256U,

    /*
     * Zero means uart_write_bytes() sends without a software TX
     * ring buffer. That is suitable for the first test.
     */
    .tx_buffer_size = 0U
};