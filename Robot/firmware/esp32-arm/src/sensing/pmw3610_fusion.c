#include "sensing/pmw3610_fusion.h"

static void pmw3610_fusion_invert2x2(const SensorCalibration *m, float inv[2][2]) {
    float det = m->m00 * m->m11 - m->m01 * m->m10;
    if (det == 0.0f) det = 1e-9f;  // degenerate calibration -- avoid a div-by-zero NaN cascade
    inv[0][0] = m->m11 / det;
    inv[0][1] = -m->m01 / det;
    inv[1][0] = -m->m10 / det;
    inv[1][1] = m->m00 / det;
}

static void pmw3610_fusion_apply_matrix(const float inv[2][2], int16_t dx, int16_t dy,
                                         float *forward_mm, float *lateral_mm) {
    *forward_mm = inv[0][0] * dx + inv[0][1] * dy;
    *lateral_mm = inv[1][0] * dx + inv[1][1] * dy;
}

void pmw3610_fusion_configure(Pmw3610Fusion *fusion, const FusionConfig *config) {
    pmw3610_fusion_invert2x2(&config->cal_left, fusion->inv_l);
    pmw3610_fusion_invert2x2(&config->cal_right, fusion->inv_r);
    fusion->baseline_mm = config->baseline_mm;
}

DeltaPose pmw3610_fusion_process(const Pmw3610Fusion *fusion, int16_t ldx, int16_t ldy, int16_t rdx,
                                  int16_t rdy) {
    float l_forward_mm, l_lateral_mm, r_forward_mm, r_lateral_mm;
    pmw3610_fusion_apply_matrix(fusion->inv_l, ldx, ldy, &l_forward_mm, &l_lateral_mm);
    pmw3610_fusion_apply_matrix(fusion->inv_r, rdx, rdy, &r_forward_mm, &r_lateral_mm);

    DeltaPose d;
    d.dtheta_rad = (r_forward_mm - l_forward_mm) / fusion->baseline_mm;
    d.dx_mm = (l_forward_mm + r_forward_mm) / 2.0f;
    // Sensor dy's raw sign is a fixed property of the hardware, independent
    // of calibration -- and it came out backwards from intuition: pushing
    // the rig's physical right empirically showed position moving left.
    // Flipped here to match the PMW3610 sensor-node project's LATERAL_SIGN fix.
    d.dy_mm = -(l_lateral_mm + r_lateral_mm) / 2.0f;
    return d;
}
