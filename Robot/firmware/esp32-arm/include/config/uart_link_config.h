/* Exposes the arm board's UART link configurations: one to the drivetrain
 * controller, one to the Raspberry Pi. */
#pragma once

#include <robot_common/uart_link.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configures the UART connection from the arm controller to the drivetrain controller.
extern const UartLinkConfig DRIVETRAIN_UART_LINK_CONFIG;

// Configures the UART connection from the arm controller to the Raspberry Pi.
extern const UartLinkConfig PI_UART_LINK_CONFIG;

#ifdef __cplusplus
}
#endif
