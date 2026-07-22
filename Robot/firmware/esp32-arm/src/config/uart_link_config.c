/* Defines the top/arm board's two physical UART links. */
#include "config/uart_link_config.h"

#include "config/pin_map.h"

// Uses UART1 for all framed traffic exchanged with the drivetrain ESP32.
const UartLinkConfig DRIVETRAIN_UART_LINK_CONFIG = {
    .uart_num = UART_NUM_1,
    .tx_pin = PIN_DRIVETRAIN_UART_TX,
    .rx_pin = PIN_DRIVETRAIN_UART_RX,
    .baud_rate = 115200U,
    .rx_buffer_size = 256U,
    .tx_buffer_size = 256U,
};

// Uses UART2 for the electrically separate Raspberry Pi connection.
// The application should initialize this link when the Pi packet protocol is integrated.
const UartLinkConfig PI_UART_LINK_CONFIG = {
    .uart_num = UART_NUM_2,
    .tx_pin = PIN_PI_UART_TX,
    .rx_pin = PIN_PI_UART_RX,
    .baud_rate = 115200U,
    .rx_buffer_size = 256U,
    .tx_buffer_size = 256U,
};
