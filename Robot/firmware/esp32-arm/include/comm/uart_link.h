#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "sensing/pmw3610_fusion.h"

// Producing side of the UART link to esp32-drivetrain (the main-control
// board). Emits this cycle's fused optical displacement (forward_mm,
// lateral_mm, dtheta_deg) as a DELTA,... line, not an integrated pose --
// esp32-drivetrain compares this against encoder-predicted displacement
// each cycle for slip detection, and needs the per-cycle delta for that.
// `valid` is this cycle's hardware validity (l_valid && r_valid) as its
// own field rather than folded into the numbers -- a real fused delta can
// legitimately be 0 or negative, so encoding "invalid" as a magic sentinel
// value would be ambiguous.

#ifdef __cplusplus
extern "C" {
#endif

void uart_link_init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud);
void uart_link_send_delta(const DeltaPose *delta, bool valid);

#ifdef __cplusplus
}
#endif
