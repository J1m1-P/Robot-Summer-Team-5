#include <Arduino.h>

#include "config/pin_map.h"
#include <robot_common/app_log.h>
#include "drivers/pmw3610_driver.h"

// Calibration-only firmware: it publishes raw deltas for the laptop helper
// and intentionally contains no fitting, persistence, or config logic.
static DualPmw3610 sensors;

void setup() {
    Serial.begin(115200);
    delay(2000);
    app_log_init();

    const PmwPinConfig pins = {
        .sdio_pin = PIN_PMW_SDIO,
        .sclk_pin = PIN_PMW_SCLK,
        .ncs_l_pin = PIN_PMW_NCS_L,
        .ncs_r_pin = PIN_PMW_NCS_R,
    };
    dual_pmw3610_init(&sensors, &pins);
}

void loop() {
    int16_t ldx = 0, ldy = 0, rdx = 0, rdy = 0;
    bool l_valid = false, r_valid = false;
    dual_pmw3610_poll(&sensors, &ldx, &ldy, &l_valid, &rdx, &rdy, &r_valid);

    Serial.printf("CAL,L,%d,%d,%d\n", ldx, ldy, l_valid ? 1 : 0);
    Serial.printf("CAL,R,%d,%d,%d\n", rdx, rdy, r_valid ? 1 : 0);
    delay(10);
}
