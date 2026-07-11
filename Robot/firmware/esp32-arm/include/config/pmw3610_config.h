#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Pins for one DualPmw3610 instance -- passed explicitly rather than
// hardcoded, so the same driver code works regardless of which main*.cpp
// wires it up (see pin_map.h for this board's actual GPIO numbers).
typedef struct {
    uint8_t sdio_pin;
    uint8_t sclk_pin;
    uint8_t ncs_l_pin;
    uint8_t ncs_r_pin;
} PmwPinConfig;

// Operating CPI is a runtime variable, not a compile-time constant --
// changeable without recompiling (e.g. from a serial command in any of
// the three mains). pmw3610_get_res_step() derives the sensor's RES_STEP
// register value from whatever CPI is currently set; RES_STEP steps are
// 200 CPI apart (0x1=200 ... 0x10=3200).
#define PMW3610_DEFAULT_CPI 3200.0f

void pmw3610_set_cpi(float cpi);
float pmw3610_get_cpi(void);
uint8_t pmw3610_cpi_to_res_step(float cpi);
uint8_t pmw3610_get_res_step(void);

// Minimum SQUAL (surface quality / feature count) for a reading to be
// trusted -- empirically determined (see PMW3610 project's PMW3610.h).
#define PMW3610_SQUAL_MIN 10

// Datasheet's fixed threshold (page 19) for the surface-coverage "Smart"
// algorithm -- shutter's low byte compared against 45 clock cycles.
#define PMW3610_SMART_SHUTTER_THRESHOLD 45

// How often (in poll cycles) dual_pmw3610_poll() checks shutter and runs
// the Smart-mode toggle, keeping that extra bus time out of most cycles.
#define PMW3610_SMART_MODE_CHECK_INTERVAL 10

#ifdef __cplusplus
}
#endif
