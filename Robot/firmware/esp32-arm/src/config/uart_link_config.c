/* Defines the arm board's UART peripherals, pins, speed, and buffer sizes. */
#include "config/uart_link_config.h"

#include "config/pin_map.h"

// UART settings used to send arm data to the drivetrain controller.
const UartLinkConfig DRIVETRAIN_UART_LINK_CONFIG = {
    .uart_num = UART_NUM_1,
    .tx_pin = PIN_DRIVETRAIN_UART_TX,
    .rx_pin = PIN_DRIVETRAIN_UART_RX,
    .baud_rate = 115200U,
    .rx_buffer_size = 256U,
    .tx_buffer_size = 256U,
};

// UART settings used to talk to the Raspberry Pi. A separate peripheral from
// the drivetrain link above -- UART1 is already in use for that connection.
// Baud must match SERIAL_BAUD in the Pi's uart_link.py.
const UartLinkConfig PI_UART_LINK_CONFIG = {
    .uart_num = UART_NUM_2,
    .tx_pin = PIN_PI_UART_TX,
    .rx_pin = PIN_PI_UART_RX,
    .baud_rate = 115200U,
    .rx_buffer_size = 256U,
    .tx_buffer_size = 256U,
};
