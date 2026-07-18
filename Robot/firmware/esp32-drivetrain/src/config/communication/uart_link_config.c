/* Defines the drivetrain UART peripheral, pins, speed, and buffer sizes. */
#include "config/communication/uart_link_config.h"

#include "config/pin_map.h"

// UART settings used to communicate with the arm controller.
const UartLinkConfig TOP_ESP_UART_LINK_CONFIG = {
    .uart_num = UART_NUM_1,
    .tx_pin = PIN_TOP_ESP32_UART_TX,
    .rx_pin = PIN_TOP_ESP32_UART_RX,
    .baud_rate = 115200U,
    .rx_buffer_size = 256U,
    .tx_buffer_size = 256U,
};
