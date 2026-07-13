#include "sensing/pmw3610_fusion.h"

#include <math.h>
#include <string.h>

static void pmw3610_fusion_invert2x2(const SensorCalibration *m, float inv[2][2]) {
    float det = m->m00 * m->m11 - m->m01 * m->m10;
    inv[0][0] = m->m11 / det;
    inv[0][1] = -m->m01 / det;
    inv[1][0] = -m->m10 / det;
    inv[1][1] = m->m00 / det;
}

static bool pmw3610_fusion_matrix_valid(const SensorCalibration *matrix) {
    if (matrix == NULL || !isfinite(matrix->m00) || !isfinite(matrix->m01) ||
        !isfinite(matrix->m10) || !isfinite(matrix->m11)) {
        return false;
    }

    float determinant = matrix->m00 * matrix->m11 - matrix->m01 * matrix->m10;
    return isfinite(determinant) && fabsf(determinant) > 1e-6f;
}

static void pmw3610_fusion_apply_matrix(const float inv[2][2], int16_t dx, int16_t dy,
                                         float *forward_mm, float *lateral_mm) {
    *forward_mm = inv[0][0] * dx + inv[0][1] * dy;
    *lateral_mm = inv[1][0] * dx + inv[1][1] * dy;
}

bool pmw3610_fusion_configure(Pmw3610Fusion *fusion, const FusionConfig *config) {
    if (fusion == NULL) return false;
    memset(fusion, 0, sizeof(*fusion));

    if (config == NULL || !isfinite(config->baseline_mm) || config->baseline_mm <= 0.0f ||
        !pmw3610_fusion_matrix_valid(&config->cal_left) ||
        !pmw3610_fusion_matrix_valid(&config->cal_right)) {
        return false;
    }

    pmw3610_fusion_invert2x2(&config->cal_left, fusion->inv_l);
    pmw3610_fusion_invert2x2(&config->cal_right, fusion->inv_r);
    fusion->baseline_mm = config->baseline_mm;
    fusion->configured = true;
    return true;
}

DeltaPose pmw3610_fusion_process(const Pmw3610Fusion *fusion, int16_t ldx, int16_t ldy, int16_t rdx,
                                  int16_t rdy) {
    DeltaPose d = {0};
    if (fusion == NULL || !fusion->configured) return d;

    float l_forward_mm, l_lateral_mm, r_forward_mm, r_lateral_mm;
    pmw3610_fusion_apply_matrix(fusion->inv_l, ldx, ldy, &l_forward_mm, &l_lateral_mm);
    pmw3610_fusion_apply_matrix(fusion->inv_r, rdx, rdy, &r_forward_mm, &r_lateral_mm);

    d.dtheta_rad = (r_forward_mm - l_forward_mm) / fusion->baseline_mm;
    d.dx_mm = (l_forward_mm + r_forward_mm) / 2.0f;
    // Sensor dy's raw sign is a fixed property of the hardware, independent
    // of calibration -- and it came out backwards from intuition: pushing
    // the rig's physical right empirically showed position moving left.
    // Flipped here to match the PMW3610 sensor-node project's LATERAL_SIGN fix.
    d.dy_mm = -(l_lateral_mm + r_lateral_mm) / 2.0f;
    return d;
}
