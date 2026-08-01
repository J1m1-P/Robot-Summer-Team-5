/*
 * RX-only counterpart to pi_uart_only_test_main.cpp -- see platformio.ini's
 * [env:pi-uart-rx-only-test].
 *
 * The round-trip harness showed every reply invisible to the ESP until its
 * own next TX on pi_uart, regardless of the request interval used. That
 * timing is equally consistent with two different bugs: (a) this ESP's TX
 * call is somehow what services its own stuck RX path, or (b) the Pi's
 * reply write isn't physically leaving the wire promptly despite its own
 * flush() call succeeding, and only transmits when something else on the
 * Pi's side nudges it -- both alternate on the same cadence, so the
 * round-trip test alone can't tell them apart.
 *
 * This harness removes the ambiguity: it never transmits on pi_uart at all
 * after boot. Run it against a companion Pi script (uart_push_test.py) that
 * pushes unsolicited PiReportPacket-shaped frames on its own timer, not in
 * response to anything from the ESP. If those frames still show up late or
 * stuck here, the ESP's own TX activity cannot be the trigger and the bug
 * must be on the Pi's transmit side or the physical wire.
 */
#include <Arduino.h>
#include "driver/uart.h"

#include <robot_common/pi_action_packet.h>
#include <robot_common/uart_link.h>

#include "config/uart_link_config.h"

namespace {

constexpr uint32_t kStatsPeriodMs = 1000U;

UartLink pi_uart = {};
uint32_t last_stats_ms = 0U;

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);

    const esp_err_t error = uart_link_init(&pi_uart, &PI_UART_LINK_CONFIG);
    if (error != ESP_OK) {
        Serial.printf("PIUARTRX,FAULT,uart_init,%s\n", esp_err_to_name(error));
    }
    Serial.println("PIUARTRX,READY,arm-pi-listen-only");
    Serial.println(
        "PIUARTRX,HELP,never transmits; run uart_push_test.py on the Pi");
}

void loop() {
    const uint32_t now_ms = millis();

    const esp_err_t update_error = uart_link_update(&pi_uart);
    if (update_error != ESP_OK) {
        Serial.printf("PIUARTRX,ERROR,uart_update,%s\n", esp_err_to_name(update_error));
    }

    while (uart_link_has_packet(&pi_uart)) {
        PacketFrame frame = {};
        if (uart_link_take_packet(&pi_uart, &frame) != ESP_OK) break;

        PiReportPacket report = {};
        if (pi_report_packet_decode(&frame, &report) == ESP_OK) {
            Serial.printf(
                "PIUARTRX,GOT,id=%u,result=%u,t=%lu\n",
                report.request_id, report.result,
                static_cast<unsigned long>(now_ms));
        } else {
            Serial.printf(
                "PIUARTRX,UNEXPECTED,t=%lu,type=%u,len=%u,payload=[",
                static_cast<unsigned long>(now_ms), frame.message_type,
                frame.payload_len);
            for (uint8_t i = 0U; i < frame.payload_len; ++i) {
                Serial.printf("%02X ", frame.payload[i]);
            }
            Serial.println("]");
        }
    }

    if (now_ms - last_stats_ms >= kStatsPeriodMs) {
        last_stats_ms = now_ms;
        size_t buffered_len = 0U;
        uart_get_buffered_data_len(pi_uart.config->uart_num, &buffered_len);
        Serial.printf(
            "PIUARTRX,STATS,t=%lu,received=%lu,checksum_errors=%lu,"
            "parse_errors=%lu,overwritten=%lu,buffered_now=%lu\n",
            static_cast<unsigned long>(now_ms),
            static_cast<unsigned long>(pi_uart.packets_received),
            static_cast<unsigned long>(pi_uart.checksum_errors),
            static_cast<unsigned long>(pi_uart.parse_errors),
            static_cast<unsigned long>(pi_uart.packets_overwritten),
            static_cast<unsigned long>(buffered_len));
    }

    delay(1);
}
