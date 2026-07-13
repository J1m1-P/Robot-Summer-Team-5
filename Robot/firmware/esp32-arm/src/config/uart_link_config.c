#include "config/uart_link_config.h"

#include "config/pin_map.h"

const UartLinkConfig DRIVETRAIN_UART_LINK_CONFIG = {
    .uart_num = UART_NUM_1,
    .tx_pin = PIN_DRIVETRAIN_UART_TX,
    .rx_pin = PIN_DRIVETRAIN_UART_RX,
    .baud_rate = 115200U,
    .rx_buffer_size = 256U,
    .tx_buffer_size = 256U,
};
