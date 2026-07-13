#include <Arduino.h>

#include "config/fusion_config.h"
#include "config/pin_map.h"
#include "config/pmw3610_config.h"
#include "config/static_calibration.h"
#include "debug/app_log.h"
#include "drivers/pmw3610_driver.h"
#include "sensing/pmw3610_fusion.h"
#include "sensing/pmw3610_pose.h"

// Human-readable diagnostic harness -- NOT the production build (that's
// src/main.cpp, built by the `esp32-s3-devkitm-1` env). Build/upload this
// with `pio run -e optical_debug -t upload`.
//
// Reads the FULL 10-byte burst (motion status, dx/dy, SQUAL, shutter,
// pixel min/avg/max) each cycle instead of the fast path the production
// harness uses, so it costs more bus time per poll -- fine for a slower
// human-watched debug console, not for the hot tracking loop.

static bool smart_disabled_l = false;
static bool smart_disabled_r = false;
static Pmw3610Fusion fusion;
static Pmw3610PoseManager pose;
static bool fusion_ready = false;

static void print_diagnostics(const char *label, const Pmw3610Diagnostics &d) {
    Serial.printf(
        "[%s] dx=%-5d dy=%-5d  mot=%d ovf=%d lpValid=%d lsrFault=%d rst=%d valid=%d  "
        "squal=%-3u shutter=%-4u pix(min/avg/max)=%u/%u/%u\n",
        label, d.dx, d.dy, d.status.motion, d.status.overflow, d.status.laser_power_valid,
        d.status.laser_fault, d.status.reset_triggered, pmw3610_status_valid(&d.status) ? 1 : 0,
        d.squal, d.shutter, d.pix_min, d.pix_avg, d.pix_max);
}

static void print_fused(const DeltaPose &delta, bool l_valid, bool r_valid) {
    const Pose current = pmw3610_pose_get(&pose);
    Serial.printf("[FUSED] d=(%.4f,%.4f,%.4fdeg) valid=%d pose=(%.2f,%.2f,%.2fdeg)\n", delta.dx_mm,
                  delta.dy_mm, delta.dtheta_rad * 180.0f / (float)M_PI, l_valid && r_valid ? 1 : 0,
                  current.x_mm, current.y_mm, current.heading_deg);
}

void setup() {
    Serial.begin(115200);
    delay(2000);  // let native USB CDC enumerate before printing setup diagnostics
    app_log_init();

    pmw3610_bus_init(PIN_PMW_SDIO, PIN_PMW_SCLK);
    pmw3610_bus_init_sensor(PIN_PMW_NCS_L, "L");
    pmw3610_bus_init_sensor(PIN_PMW_NCS_R, "R");

    FusionConfig config;
    fusion_ready = static_calibration_load(&config);
    if (fusion_ready) {
        fusion_ready = pmw3610_fusion_configure(&fusion, &config);
    }
    if (fusion_ready) {
        pmw3610_pose_init(&pose);
        Serial.println("--- raw diagnostics + fused delta/pose stream ---");
    } else {
        Serial.println("--- raw diagnostics stream; fusion disabled: invalid /calibration.json ---");
    }
}

void loop() {
    Pmw3610Diagnostics l = pmw3610_bus_burst_read_diagnostics(PIN_PMW_NCS_L);
    Pmw3610Diagnostics r = pmw3610_bus_burst_read_diagnostics(PIN_PMW_NCS_R);
    pmw3610_bus_apply_smart_surface_mode(PIN_PMW_NCS_L, l.shutter, &smart_disabled_l);
    pmw3610_bus_apply_smart_surface_mode(PIN_PMW_NCS_R, r.shutter, &smart_disabled_r);
    print_diagnostics("L", l);
    print_diagnostics("R", r);

    if (fusion_ready) {
        const bool l_valid = pmw3610_status_valid(&l.status);
        const bool r_valid = pmw3610_status_valid(&r.status);
        const DeltaPose delta = pmw3610_fusion_process(&fusion, l.dx, l.dy, r.dx, r.dy);
        pmw3610_pose_update(&pose, &delta, l_valid, r_valid);
        print_fused(delta, l_valid, r_valid);
    }
    delay(100);  // slower cadence -- this is for a human to read, not for logging/plotting
}
