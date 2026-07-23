/*
 * Bridges the Pi-facing UART link and the drivetrain-facing UART link: relays
 * COMMAND packets down to the drivetrain and STATUS packets back up to the Pi.
 */
#pragma once

#include <robot_common/uart_link.h>

#ifdef __cplusplus
extern "C" {
#endif

// Owns the two UART links the bridge relays packets between.
typedef struct {
    UartLink to_pi;
    UartLink to_drivetrain;
} PiBridge;

// Initializes both UART links. `bridge` must be zero-initialized before this call.
esp_err_t pi_bridge_init(PiBridge *bridge);

// Drains both links and relays: Pi COMMAND -> drivetrain, drivetrain STATUS -> Pi.
// Call every loop iteration.
esp_err_t pi_bridge_update(PiBridge *bridge);

#ifdef __cplusplus
}
#endif
