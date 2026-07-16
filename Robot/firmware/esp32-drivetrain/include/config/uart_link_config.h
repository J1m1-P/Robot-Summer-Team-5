/* Exposes the drivetrain board's UART link configuration for the arm ESP32. */
#pragma once

#include <robot_common/uart_link.h>

#ifdef __cplusplus
extern "C" {
#endif

// UART settings used to receive data from the arm controller.
extern const UartLinkConfig TOP_ESP_UART_LINK_CONFIG;

#ifdef __cplusplus
}
#endif
