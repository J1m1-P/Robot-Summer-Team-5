#include "comm/uart_link.h"

#include <math.h>
#include <stdio.h>

#include "driver/uart.h"

#define UART_LINK_PORT      UART_NUM_1
#define UART_LINK_BUF_SIZE  256

void uart_link_init(uint8_t tx_pin, uint8_t rx_pin, uint32_t baud) {
    uart_config_t config = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    uart_param_config(UART_LINK_PORT, &config);
    uart_set_pin(UART_LINK_PORT, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_LINK_PORT, UART_LINK_BUF_SIZE, 0, 0, NULL, 0);
}

void uart_link_send_delta(const DeltaPose *delta, bool valid) {
    char buf[64];
    int len = snprintf(buf, sizeof(buf), "DELTA,%.4f,%.4f,%.4f,%d\n", delta->dx_mm, delta->dy_mm,
                        delta->dtheta_rad * 180.0f / (float)M_PI, valid ? 1 : 0);
    uart_write_bytes(UART_LINK_PORT, buf, len);
}
