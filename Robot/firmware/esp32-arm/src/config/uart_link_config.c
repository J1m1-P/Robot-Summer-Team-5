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
//
// UART_NUM_2 (its default pin assignment here) was bench-tested extensively
// and found to reliably lose/delay RX by a full request cycle -- reproduced
// standalone with no other arm-board subsystem involved, independent of the
// event-queue/RX-timeout driver config. Swapping only the peripheral (same
// PIN_PI_UART_TX/RX pins, routed via the pin matrix) to UART_NUM_1 and then
// UART_NUM_0 both came back clean over 30+ cycles, isolating the fault to
// UART_NUM_2 as a peripheral instance on this chip. UART_NUM_1 is already
// used by DRIVETRAIN_UART_LINK_CONFIG above, so UART_NUM_0 is the Pi link's
// permanent home -- it's otherwise unused since Serial runs over native USB
// CDC (ARDUINO_USB_CDC_ON_BOOT=1), not the physical UART0 pins.
const UartLinkConfig PI_UART_LINK_CONFIG = {
    .uart_num = UART_NUM_0,
    .tx_pin = PIN_PI_UART_TX,
    .rx_pin = PIN_PI_UART_RX,
    .baud_rate = 115200U,
    .rx_buffer_size = 256U,
    .tx_buffer_size = 256U,
};
