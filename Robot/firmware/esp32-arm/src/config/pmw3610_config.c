#include "config/pmw3610_config.h"

#include <math.h>

static float s_cpi = PMW3610_DEFAULT_CPI;

uint8_t pmw3610_cpi_to_res_step(float cpi) {
    if (!isfinite(cpi) || cpi < 200.0f || cpi > 3200.0f) return 0U;

    float step = cpi / 200.0f;
    float rounded_step = roundf(step);
    if (fabsf(step - rounded_step) > 1e-6f) return 0U;
    return (uint8_t)rounded_step;
}

bool pmw3610_set_cpi(float cpi) {
    if (pmw3610_cpi_to_res_step(cpi) == 0U) return false;
    s_cpi = cpi;
    return true;
}

float pmw3610_get_cpi(void) {
    return s_cpi;
}

uint8_t pmw3610_get_res_step(void) {
    return pmw3610_cpi_to_res_step(s_cpi);
}
