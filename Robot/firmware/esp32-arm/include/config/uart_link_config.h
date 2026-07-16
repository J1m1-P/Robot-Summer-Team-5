/* Exposes the arm board's UART link configuration for the drivetrain connection. */
#pragma once

#include <robot_common/uart_link.h>

#ifdef __cplusplus
extern "C" {
#endif

// Configures the UART connection from the arm controller to the drivetrain controller.
extern const UartLinkConfig DRIVETRAIN_UART_LINK_CONFIG;

#ifdef __cplusplus
}
#endif
