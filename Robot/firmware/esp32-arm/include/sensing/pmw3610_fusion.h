#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "config/fusion_config.h"

// Fusion Engine: pure per-cycle math, no integration and no fault-latching
// state -- that's pmw3610_pose's job. Deliberately free of any
// hardware/ESP-IDF dependency so it stays trivially host-testable, and
// decoupled from configuration storage (pmw3610_fusion_configure() just takes
// a FusionConfig loaded from the static JSON file at boot).

#ifdef __cplusplus
extern "C" {
#endif

// Per-cycle fused output of pmw3610_fusion_process() -- body-frame
// incremental motion, not yet integrated into a cumulative pose (that's
// pmw3610_pose's job). dy_mm already has the confirmed hardware
// lateral-sign correction folded in (see pmw3610_fusion.c).
typedef struct {
    float dx_mm;      // forward
    float dy_mm;      // lateral
    float dtheta_rad;
} DeltaPose;

typedef struct {
    float inv_l[2][2];
    float inv_r[2][2];
    float baseline_mm;
    bool configured;
} Pmw3610Fusion;

// baseline_mm is the physical sensor separation (a direct caliper
// measurement, not fitted).
bool pmw3610_fusion_configure(Pmw3610Fusion *fusion, const FusionConfig *config);

// d_theta = (dx_R - dx_L) / baseline (dx is forward, dy is lateral --
// confirmed against real hardware). Pure function of this cycle's raw
// counts and the configured calibration -- no other state.
DeltaPose pmw3610_fusion_process(const Pmw3610Fusion *fusion, int16_t ldx, int16_t ldy, int16_t rdx,
                                  int16_t rdy);

#ifdef __cplusplus
}
#endif
