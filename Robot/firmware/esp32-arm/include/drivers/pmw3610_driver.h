#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config/pmw3610_config.h"

// Driver for the PMW3610 optical flow sensor over a bit-banged 3-wire SPI
// bus (SDIO is bidirectional -- not a hardware SPI peripheral). Multiple
// sensors can share one SDIO/SCLK pair; each is distinguished purely by
// its own NCS line. NCS_L and NCS_R must never both be low at once -- see
// dual_pmw3610_poll(), which brackets every transaction with NCS high/low
// so this invariant holds automatically.
//
// Register addresses and bit layouts are from the PMW3610DM-SUDU datasheet
// (PMS0003-PMW3610DM-SUDU-DS-R2.4), register map on its page 20.

#ifdef __cplusplus
extern "C" {
#endif

// Decoded MOTION register (address 0x02) bits -- the burst read's first
// byte -- plus SQUAL from the same burst. OVF, LSR_FAULT, and SQUAL are
// what's actually useful for telling a genuinely-zero reading apart from a
// bad one: OVF means the 12-bit delta counters overflowed this cycle
// (dx/dy not reliable this cycle); SQUAL below PMW3610_SQUAL_MIN means the
// sensor isn't tracking anything (e.g. lifted off the surface).
typedef struct {
    bool motion;             // bit7 MOT: motion occurred since last read
    bool overflow;            // bit4 OVF: delta counters overflowed -- dx/dy invalid this cycle
    bool laser_power_valid;   // bit3 LP_VALID
    bool laser_fault;         // bit2 LSR_FAULT: VCSEL shorted to GND/VDD
    bool reset_triggered;     // bit0 RST_FLAG
    uint8_t squal;            // surface quality / feature count -- see PMW3610_SQUAL_MIN
} Pmw3610Status;

// Whether this cycle's dx/dy should be trusted at all.
bool pmw3610_status_valid(const Pmw3610Status *status);

// Optional extra fields available from a full (10-byte) burst read, for
// diagnostics/bring-up -- not read on the hot tracking path since each
// extra byte costs bus time every poll cycle.
typedef struct {
    Pmw3610Status status;
    int16_t dx;
    int16_t dy;
    uint8_t squal;    // surface tracking quality / feature count
    uint16_t shutter; // auto-exposure shutter value (correlates with Z-height)
    uint8_t pix_max;
    uint8_t pix_avg;
    uint8_t pix_min;
} Pmw3610Diagnostics;

// Bus-level functions -- shared SDIO/SCLK pins are set once via
// pmw3610_bus_init(); every other call takes the target sensor's NCS pin
// explicitly so the bus can serve however many sensors share it.
void pmw3610_bus_init(uint8_t sdio_pin, uint8_t sclk_pin);

uint8_t pmw3610_bus_read_register(uint8_t ncs_pin, uint8_t addr);
void pmw3610_bus_write_register(uint8_t ncs_pin, uint8_t addr, uint8_t value);

// Fast path: motion + 3 delta bytes + SQUAL (5 of the burst's 10 bytes;
// SQUAL is needed for pmw3610_status_valid() to detect liftoff). Used every
// poll cycle by dual_pmw3610_poll().
void pmw3610_bus_burst_read_motion(uint8_t ncs_pin, int16_t *dx, int16_t *dy, Pmw3610Status *status);

// Full burst (all 10 bytes) for diagnostics/bring-up -- costs more bus
// time per call than pmw3610_bus_burst_read_motion, not for the hot loop.
Pmw3610Diagnostics pmw3610_bus_burst_read_diagnostics(uint8_t ncs_pin);

// Sets the sensor's resolution (see RES_STEP / pmw3610_get_res_step()).
// RES_STEP lives on SPI "page 1", so this runs the datasheet's documented
// page-switch sequence (page 36): enable SPI clock, switch to page 1,
// write RES_STEP, switch back to page 0, disable SPI clock.
void pmw3610_bus_set_resolution(uint8_t ncs_pin, uint8_t res_value);

// Datasheet page 19's "mouse level algorithm to enhance surface coverage"
// (granite/tile-type surfaces): toggles SMART_MODE based on a hysteresis
// around PMW3610_SMART_SHUTTER_THRESHOLD, following the datasheet's example
// pseudocode exactly. `smart_disabled` is the caller-owned persistent flag
// (one per sensor -- see DualPmw3610) matching the datasheet's
// SW_SMART_Flag; true means "currently disabled, watching for shutter to
// drop back below threshold", false means "currently enabled, watching for
// shutter to climb above it".
//
// Two entry points: pmw3610_bus_update_smart_surface_mode() reads
// SHUTTER_HIGHER/LOWER itself (2 extra register reads) for callers that
// don't already have a fresh shutter value; pmw3610_bus_apply_smart_surface_mode()
// skips those reads for callers that already fetched shutter some other
// way (e.g. a full diagnostics burst already includes it).
void pmw3610_bus_update_smart_surface_mode(uint8_t ncs_pin, bool *smart_disabled);
void pmw3610_bus_apply_smart_surface_mode(uint8_t ncs_pin, uint16_t shutter, bool *smart_disabled);

// Runs the datasheet SPI-port reset sequence, product-ID read, self-test,
// recommended power-management config, and resolution setting for one
// sensor. Logs diagnostics via APP_LOG tagged LOG_TAG_PMW3610, labeled with
// `label` (e.g. "L"/"R").
void pmw3610_bus_init_sensor(uint8_t ncs_pin, const char *label);

// Convenience wrapper for a two-sensor layout. Owns the alternating
// L-first/R-first read order (removes systematic timing bias between the
// two sensors) so callers don't have to re-implement it.
typedef struct {
    PmwPinConfig pins;
    bool left_first;

    // Surface-coverage Smart-mode state (see pmw3610_bus_update_smart_surface_mode).
    // Always on; checked every PMW3610_SMART_MODE_CHECK_INTERVAL poll
    // cycles rather than every cycle, so the hot burst-read path's timing
    // is unaffected.
    bool smart_disabled_l;
    bool smart_disabled_r;
    uint16_t poll_count;
} DualPmw3610;

void dual_pmw3610_init(DualPmw3610 *dual, const PmwPinConfig *pins);

// l_valid/r_valid are hardware-confirmed (OVF/LSR_FAULT/SQUAL derived --
// see pmw3610_status_valid()) validity for this cycle's dx/dy -- callers
// should discard/hold last-good rather than integrate a cycle whose valid
// flag is false.
void dual_pmw3610_poll(DualPmw3610 *dual, int16_t *ldx, int16_t *ldy, bool *l_valid, int16_t *rdx,
                        int16_t *rdy, bool *r_valid);

#ifdef __cplusplus
}
#endif
