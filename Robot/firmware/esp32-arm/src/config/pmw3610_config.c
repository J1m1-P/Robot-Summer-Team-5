#include "config/pmw3610_config.h"

static float s_cpi = PMW3610_DEFAULT_CPI;

uint8_t pmw3610_cpi_to_res_step(float cpi) {
    return (uint8_t)(cpi / 200.0f);
}

void pmw3610_set_cpi(float cpi) {
    s_cpi = cpi;
}

float pmw3610_get_cpi(void) {
    return s_cpi;
}

uint8_t pmw3610_get_res_step(void) {
    return pmw3610_cpi_to_res_step(s_cpi);
}
