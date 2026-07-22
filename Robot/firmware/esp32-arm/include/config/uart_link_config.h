/* Exposes the arm board's independent UART configurations for its two peers. */
#pragma once

#include <robot_common/uart_link.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configures the UART connection from the arm controller to the drivetrain controller.
extern const UartLinkConfig DRIVETRAIN_UART_LINK_CONFIG;

// Reserves a separate peripheral and pins for the Raspberry Pi connection.
extern const UartLinkConfig PI_UART_LINK_CONFIG;

#ifdef __cplusplus
}
#endif
