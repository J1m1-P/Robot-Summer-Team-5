/* Minimal PMW3610-to-drivetrain odometry link bench harness -- see
 * platformio.ini's [env:odometry-link-test].
 *
 * Isolated from production main.cpp on purpose: no ToF, no task-link
 * client/server. Just PMW3610 fusion + the UART odometry producer, so
 * verifying the drivetrain-board fusion path doesn't depend on unrelated
 * hardware (ToF sensors, the Pi link, etc.) also being present and working.
 *
 * Usage: flash this env, open its serial monitor. It prints the producer's
 * readiness once at boot, then a periodic status line with the cumulative
 * pose and fault status so you can confirm packets are flowing and the pose
 * moves plausibly when the sensors are moved by hand.
 */
#include <Arduino.h>

#include "comm/odometry_link_producer.h"
#include "config/pin_map.h"
#include "config/uart_link_config.h"

static UartLink drivetrain_uart = {};
static OdometryLinkProducer producer = {};
static unsigned long last_print_ms = 0;
static const unsigned long kPrintPeriodMs = 250;

void setup() {
    Serial.begin(115200);
    delay(1000);

    ESP_ERROR_CHECK(uart_link_init(&drivetrain_uart, &DRIVETRAIN_UART_LINK_CONFIG));

    const PmwPinConfig pins = {
        .sdio_pin = PIN_PMW_SDIO,
        .sclk_pin = PIN_PMW_SCLK,
        .ncs_l_pin = PIN_PMW_NCS_L,
        .ncs_r_pin = PIN_PMW_NCS_R,
    };
    if (!odometry_link_producer_init(&producer, &pins)) {
        Serial.println("# odometry_link_producer_init FAILED -- missing or "
                       "invalid /calibration.json; no packets will be sent");
    } else {
        Serial.println("# odometry_link_producer_init OK -- streaming "
                       "odometry packets to the drivetrain board");
    }
}

void loop() {
    const esp_err_t error = odometry_link_producer_update(&producer, &drivetrain_uart);
    if (error != ESP_OK) {
        Serial.print("# odometry_link_producer_update failed: ");
        Serial.println(esp_err_to_name(error));
    }

    const unsigned long now_ms = millis();
    if (now_ms - last_print_ms >= kPrintPeriodMs) {
        last_print_ms = now_ms;
        const Pose pose = pmw3610_pose_get(&producer.pose);
        const PoseStatus status = pmw3610_pose_get_status(&producer.pose);
        Serial.printf(
            "seq=%lu ready=%d pose=(%.2f,%.2f,%.2fdeg) output_killed=%d "
            "last_fault=%d l_valid=%d r_valid=%d\n",
            (unsigned long)producer.sequence, producer.ready ? 1 : 0,
            pose.x_mm, pose.y_mm, pose.heading_deg,
            status.output_killed ? 1 : 0, (int)status.last_fault,
            producer.last_l_valid ? 1 : 0, producer.last_r_valid ? 1 : 0);
    }
    // Matches production main.cpp's own pacing exactly (see its loop()) --
    // this bench harness previously used a much slower, arbitrary 10ms
    // delay that had nothing to do with sensor timing and just throttled
    // the refresh rate well below what production actually achieves.
    delay(1);
}
