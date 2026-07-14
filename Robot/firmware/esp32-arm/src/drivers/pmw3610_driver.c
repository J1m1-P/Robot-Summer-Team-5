#include "drivers/pmw3610_driver.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_rom_sys.h"

#include "config/pmw3610_config.h"
#include "debug/app_log.h"

// Register addresses and bit layouts are from the PMW3610DM-SUDU datasheet
// (PMS0003-PMW3610DM-SUDU-DS-R2.4), register map on its page 20. File-local
// only -- nothing outside this driver needs raw register addresses.
#define PMW3610_REG_PROD_ID          0x00
#define PMW3610_REG_MOTION           0x02
#define PMW3610_REG_DELTA_X_L        0x03
#define PMW3610_REG_DELTA_Y_L        0x04
#define PMW3610_REG_DELTA_XY_H       0x05
#define PMW3610_REG_SQUAL            0x06
#define PMW3610_REG_SHUTTER_HIGHER   0x07
#define PMW3610_REG_SHUTTER_LOWER    0x08
#define PMW3610_REG_PIX_MAX          0x09
#define PMW3610_REG_PIX_AVG          0x0a
#define PMW3610_REG_PIX_MIN          0x0b
#define PMW3610_REG_PERFORMANCE      0x11
#define PMW3610_REG_BURST_READ       0x12
#define PMW3610_REG_RUN_DOWNSHIFT    0x1b
#define PMW3610_REG_REST1_RATE       0x1c
#define PMW3610_REG_REST1_DOWNSHIFT  0x1d
#define PMW3610_REG_OBSERVATION1     0x2d
#define PMW3610_REG_SPI_CLK_ON_REQ   0x41
#define PMW3610_REG_SPI_PAGE0        0x7f  // also doubles as SPI_PAGE1 selector at 0xff
#define PMW3610_REG_RES_STEP         0x85  // page-1 only: CPI + SWAPXY/INV_X/INV_Y
#define PMW3610_REG_SMART_MODE       0x32

static uint8_t s_sdio_pin = 0;
static uint8_t s_sclk_pin = 0;

static void pmw3610_gpio_set_output(uint8_t pin) {
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&config);
}

static void pmw3610_gpio_set_input(uint8_t pin) {
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&config);
}

static void pmw3610_write_byte(uint8_t value) {
    pmw3610_gpio_set_output(s_sdio_pin);
    for (int8_t i = 7; i >= 0; i--) {
        gpio_set_level((gpio_num_t)s_sdio_pin, (value >> i) & 1);
        esp_rom_delay_us(1);
        gpio_set_level((gpio_num_t)s_sclk_pin, 1);  // sensor samples on rising edge
        esp_rom_delay_us(1);
        gpio_set_level((gpio_num_t)s_sclk_pin, 0);
        esp_rom_delay_us(1);
    }
}

static uint8_t pmw3610_read_byte(void) {
    uint8_t value = 0;
    for (int8_t i = 7; i >= 0; i--) {
        gpio_set_level((gpio_num_t)s_sclk_pin, 1);  // sensor shifts new bit out after prior LOW
        esp_rom_delay_us(1);
        value |= (gpio_get_level((gpio_num_t)s_sdio_pin) << i);
        gpio_set_level((gpio_num_t)s_sclk_pin, 0);
        esp_rom_delay_us(1);
    }
    return value;
}

static Pmw3610Status pmw3610_decode_status(uint8_t motion_byte, uint8_t squal) {
    Pmw3610Status s;
    s.motion = motion_byte & 0x80;
    s.overflow = motion_byte & 0x10;
    s.laser_power_valid = motion_byte & 0x08;
    s.laser_fault = motion_byte & 0x04;
    s.reset_triggered = motion_byte & 0x01;
    s.squal = squal;
    return s;
}

bool pmw3610_status_valid(const Pmw3610Status *status) {
    return !status->overflow && status->laser_power_valid && !status->laser_fault &&
           !status->reset_triggered && status->squal >= PMW3610_SQUAL_MIN;
}

void pmw3610_bus_init(uint8_t sdio_pin, uint8_t sclk_pin) {
    s_sdio_pin = sdio_pin;
    s_sclk_pin = sclk_pin;
    pmw3610_gpio_set_output(s_sclk_pin);
    gpio_set_level((gpio_num_t)s_sclk_pin, 0);
}

uint8_t pmw3610_bus_read_register(uint8_t ncs_pin, uint8_t addr) {
    gpio_set_level((gpio_num_t)ncs_pin, 0);
    pmw3610_write_byte(addr & 0x7F);  // MSB=0 for read
    pmw3610_gpio_set_input(s_sdio_pin);
    esp_rom_delay_us(5);              // required turnaround
    uint8_t val = pmw3610_read_byte();
    gpio_set_level((gpio_num_t)ncs_pin, 1);
    esp_rom_delay_us(2);
    return val;
}

void pmw3610_bus_write_register(uint8_t ncs_pin, uint8_t addr, uint8_t value) {
    gpio_set_level((gpio_num_t)ncs_pin, 0);
    pmw3610_write_byte(addr | 0x80);  // MSB=1 for write
    pmw3610_write_byte(value);
    gpio_set_level((gpio_num_t)ncs_pin, 1);
    esp_rom_delay_us(2);
}

void pmw3610_bus_burst_read_motion(uint8_t ncs_pin, int16_t *dx, int16_t *dy, Pmw3610Status *status) {
    gpio_set_level((gpio_num_t)ncs_pin, 0);
    pmw3610_write_byte(PMW3610_REG_BURST_READ);
    pmw3610_gpio_set_input(s_sdio_pin);
    esp_rom_delay_us(5);
    uint8_t motion = pmw3610_read_byte();
    uint8_t dx_l = pmw3610_read_byte();
    uint8_t dy_l = pmw3610_read_byte();
    uint8_t dxy_h = pmw3610_read_byte();
    uint8_t squal = pmw3610_read_byte();
    gpio_set_level((gpio_num_t)ncs_pin, 1);  // terminates burst (t_BEXIT)
    esp_rom_delay_us(2);

    *status = pmw3610_decode_status(motion, squal);

    int16_t x = ((dxy_h >> 4) << 8) | dx_l;
    int16_t y = ((dxy_h & 0x0F) << 8) | dy_l;
    if (x >= 2048) x -= 4096;
    if (y >= 2048) y -= 4096;
    *dx = x;
    *dy = y;
}

Pmw3610Diagnostics pmw3610_bus_burst_read_diagnostics(uint8_t ncs_pin) {
    gpio_set_level((gpio_num_t)ncs_pin, 0);
    pmw3610_write_byte(PMW3610_REG_BURST_READ);
    pmw3610_gpio_set_input(s_sdio_pin);
    esp_rom_delay_us(5);
    uint8_t motion = pmw3610_read_byte();
    uint8_t dx_l = pmw3610_read_byte();
    uint8_t dy_l = pmw3610_read_byte();
    uint8_t dxy_h = pmw3610_read_byte();
    uint8_t squal = pmw3610_read_byte();
    uint8_t shutter_hi = pmw3610_read_byte();
    uint8_t shutter_lo = pmw3610_read_byte();
    uint8_t pix_max = pmw3610_read_byte();
    uint8_t pix_avg = pmw3610_read_byte();
    uint8_t pix_min = pmw3610_read_byte();
    gpio_set_level((gpio_num_t)ncs_pin, 1);  // terminates burst (t_BEXIT)
    esp_rom_delay_us(2);

    Pmw3610Diagnostics d;
    d.status = pmw3610_decode_status(motion, squal);

    int16_t x = ((dxy_h >> 4) << 8) | dx_l;
    int16_t y = ((dxy_h & 0x0F) << 8) | dy_l;
    if (x >= 2048) x -= 4096;
    if (y >= 2048) y -= 4096;
    d.dx = x;
    d.dy = y;
    d.squal = squal;
    d.shutter = ((uint16_t)shutter_hi << 8) | shutter_lo;
    d.pix_max = pix_max;
    d.pix_avg = pix_avg;
    d.pix_min = pix_min;
    return d;
}

void pmw3610_bus_set_resolution(uint8_t ncs_pin, uint8_t res_value) {
    // Datasheet page 36: RES_STEP (0x85) lives on SPI page 1, requiring the
    // documented clock-enable + page-switch sequence around the write.
    pmw3610_bus_write_register(ncs_pin, PMW3610_REG_SPI_CLK_ON_REQ, 0xBA);  // enable SPI clock
    esp_rom_delay_us(300);                                                  // datasheet: wait 300us
    pmw3610_bus_write_register(ncs_pin, PMW3610_REG_SPI_PAGE0, 0xFF);       // switch to page 1
    pmw3610_bus_write_register(ncs_pin, PMW3610_REG_RES_STEP, res_value);
    pmw3610_bus_write_register(ncs_pin, PMW3610_REG_SPI_PAGE0, 0x00);       // switch back to page 0
    pmw3610_bus_write_register(ncs_pin, PMW3610_REG_SPI_CLK_ON_REQ, 0xB5);  // disable SPI clock
}

void pmw3610_bus_apply_smart_surface_mode(uint8_t ncs_pin, uint16_t shutter, bool *smart_disabled) {
    // Datasheet page 19 pseudocode, applied literally: only acts when the
    // shutter high byte is exactly zero (i.e. shutter < 256).
    //   if (SW_SMART_Flag && shutter.hi==0 && shutter.lo<45)  { enable;  flag=0; }
    //   if (!SW_SMART_Flag && shutter.hi==0 && shutter.lo>45) { disable; flag=1; }
    uint8_t shutter_hi = shutter >> 8;
    uint8_t shutter_lo = shutter & 0xFF;

    if (*smart_disabled && shutter_hi == 0 && shutter_lo < PMW3610_SMART_SHUTTER_THRESHOLD) {
        pmw3610_bus_write_register(ncs_pin, PMW3610_REG_SPI_CLK_ON_REQ, 0xBA);
        pmw3610_bus_write_register(ncs_pin, PMW3610_REG_SMART_MODE, 0x00);  // enable
        pmw3610_bus_write_register(ncs_pin, PMW3610_REG_SPI_CLK_ON_REQ, 0xB5);
        *smart_disabled = false;
    } else if (!*smart_disabled && shutter_hi == 0 && shutter_lo > PMW3610_SMART_SHUTTER_THRESHOLD) {
        pmw3610_bus_write_register(ncs_pin, PMW3610_REG_SPI_CLK_ON_REQ, 0xBA);
        pmw3610_bus_write_register(ncs_pin, PMW3610_REG_SMART_MODE, 0x80);  // disable
        pmw3610_bus_write_register(ncs_pin, PMW3610_REG_SPI_CLK_ON_REQ, 0xB5);
        *smart_disabled = true;
    }
}

void pmw3610_bus_update_smart_surface_mode(uint8_t ncs_pin, bool *smart_disabled) {
    uint8_t shutter_hi = pmw3610_bus_read_register(ncs_pin, PMW3610_REG_SHUTTER_HIGHER);
    uint8_t shutter_lo = pmw3610_bus_read_register(ncs_pin, PMW3610_REG_SHUTTER_LOWER);
    uint16_t shutter = ((uint16_t)shutter_hi << 8) | shutter_lo;
    pmw3610_bus_apply_smart_surface_mode(ncs_pin, shutter, smart_disabled);
}

void pmw3610_bus_init_sensor(uint8_t ncs_pin, const char *label) {
    pmw3610_gpio_set_output(ncs_pin);
    gpio_set_level((gpio_num_t)ncs_pin, 1);

    // SPI port reset sequence (per datasheet).
    gpio_set_level((gpio_num_t)ncs_pin, 1);
    esp_rom_delay_us(10);
    gpio_set_level((gpio_num_t)ncs_pin, 0);
    esp_rom_delay_us(10);
    gpio_set_level((gpio_num_t)ncs_pin, 1);
    esp_rom_delay_us(200);  // >=150us, one frame time

    uint8_t prod_id = pmw3610_bus_read_register(ncs_pin, PMW3610_REG_PROD_ID);
    APP_LOGI(LOG_TAG_PMW3610, "[%s] Product ID: 0x%02X (expect 0x3E)", label, prod_id);

    // self-test
    pmw3610_bus_write_register(ncs_pin, PMW3610_REG_OBSERVATION1, 0x00);
    esp_rom_delay_us(10000);
    uint8_t obs = pmw3610_bus_read_register(ncs_pin, PMW3610_REG_OBSERVATION1);
    APP_LOGI(LOG_TAG_PMW3610, "[%s] Self-test (0x2D): 0x%02X (low nibble should be 0xF)", label, obs);

    // recommended power-management config (datasheet power-up procedure)
    pmw3610_bus_write_register(ncs_pin, PMW3610_REG_PERFORMANCE, 0x0D);
    pmw3610_bus_write_register(ncs_pin, PMW3610_REG_RUN_DOWNSHIFT, 0x04);
    pmw3610_bus_write_register(ncs_pin, PMW3610_REG_REST1_RATE, 0x04);
    pmw3610_bus_write_register(ncs_pin, PMW3610_REG_REST1_DOWNSHIFT, 0x0F);

    // Explicit resolution -- removes reliance on the unconfirmed power-on default.
    uint8_t res_step = pmw3610_get_res_step();
    pmw3610_bus_set_resolution(ncs_pin, res_step);
    APP_LOGI(LOG_TAG_PMW3610, "[%s] Resolution set to 0x%02X (%.0f CPI)", label, res_step,
             pmw3610_get_cpi());
}

void dual_pmw3610_init(DualPmw3610 *dual, const PmwPinConfig *pins) {
    memset(dual, 0, sizeof(*dual));
    dual->pins = *pins;
    dual->left_first = true;

    pmw3610_bus_init(pins->sdio_pin, pins->sclk_pin);
    pmw3610_bus_init_sensor(pins->ncs_l_pin, "L");
    pmw3610_bus_init_sensor(pins->ncs_r_pin, "R");
}

bool dual_pmw3610_set_cpi(DualPmw3610 *dual, float cpi) {
    if (dual == NULL || !pmw3610_set_cpi(cpi)) return false;

    uint8_t resolution_step = pmw3610_get_res_step();
    pmw3610_bus_set_resolution(dual->pins.ncs_l_pin, resolution_step);
    pmw3610_bus_set_resolution(dual->pins.ncs_r_pin, resolution_step);
    return true;
}

void dual_pmw3610_poll(DualPmw3610 *dual, int16_t *ldx, int16_t *ldy, bool *l_valid, int16_t *rdx,
                        int16_t *rdy, bool *r_valid) {
    Pmw3610Status l_status, r_status;
    if (dual->left_first) {
        pmw3610_bus_burst_read_motion(dual->pins.ncs_l_pin, ldx, ldy, &l_status);
        pmw3610_bus_burst_read_motion(dual->pins.ncs_r_pin, rdx, rdy, &r_status);
    } else {
        pmw3610_bus_burst_read_motion(dual->pins.ncs_r_pin, rdx, rdy, &r_status);
        pmw3610_bus_burst_read_motion(dual->pins.ncs_l_pin, ldx, ldy, &l_status);
    }
    dual->left_first = !dual->left_first;
    *l_valid = pmw3610_status_valid(&l_status);
    *r_valid = pmw3610_status_valid(&r_status);

    if (l_status.reset_triggered) {
        pmw3610_bus_init_sensor(dual->pins.ncs_l_pin, "L");
    }
    if (r_status.reset_triggered) {
        pmw3610_bus_init_sensor(dual->pins.ncs_r_pin, "R");
    }

    // Surface-coverage Smart-mode: a separate, lower-cadence check (see
    // PMW3610_SMART_MODE_CHECK_INTERVAL) rather than every cycle, so this
    // doesn't add bus time to the hot burst-read path above.
    dual->poll_count++;
    if (dual->poll_count >= PMW3610_SMART_MODE_CHECK_INTERVAL) {
        dual->poll_count = 0;
        pmw3610_bus_update_smart_surface_mode(dual->pins.ncs_l_pin, &dual->smart_disabled_l);
        pmw3610_bus_update_smart_surface_mode(dual->pins.ncs_r_pin, &dual->smart_disabled_r);
    }
}
