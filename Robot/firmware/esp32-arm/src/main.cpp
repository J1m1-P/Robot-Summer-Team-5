/* Runs the arm board's production sensor loop and streams fused PMW3610
 * odometry to the drivetrain board over UART. */
#include <Arduino.h>

#include <robot_common/uart_link.h>

#include "comm/odometry_link_producer.h"
#include "config/pin_map.h"
#include "config/tof_config.h"
#include "config/uart_link_config.h"
#include "control/time_of_flight/tof_manager.h"

static TofManager tof_manager = {};
static UartLink drivetrain_uart = {}; /**< Physical link to drivetrain ESP32. */
static OdometryLinkProducer odometry_producer = {}; /**< Streams fused PMW3610 odometry. */

// Initializes all three claw sensors, begins continuous ranging, and starts
// the drivetrain UART link + PMW3610 odometry producer.
void setup() {
    ESP_ERROR_CHECK(tof_manager_init(&tof_manager, &ARM_TOF_CONFIG));
    ESP_ERROR_CHECK(tof_manager_start(&tof_manager));

    ESP_ERROR_CHECK(uart_link_init(&drivetrain_uart,
                                   &DRIVETRAIN_UART_LINK_CONFIG));

    const PmwPinConfig pmw_pins = {
        .sdio_pin = PIN_PMW_SDIO,
        .sclk_pin = PIN_PMW_SCLK,
        .ncs_l_pin = PIN_PMW_NCS_L,
        .ncs_r_pin = PIN_PMW_NCS_R,
    };
    if (!odometry_link_producer_init(&odometry_producer, &pmw_pins)) {
        Serial.println("# odometry_link_producer_init FAILED -- missing or "
                       "invalid calibration; drivetrain falls back to its "
                       "other odometry sources");
    }
}

// Refreshes each sensor's cached sample without blocking for measurements,
// and streams the latest fused odometry packet to the drivetrain board.
void loop() {
    const esp_err_t error = tof_manager_poll(&tof_manager);
    if (error != ESP_OK && error != ESP_ERR_NOT_FINISHED) {
        ESP_ERROR_CHECK(error);
    }
    (void)odometry_link_producer_update(&odometry_producer, &drivetrain_uart);
    delay(1);
}
