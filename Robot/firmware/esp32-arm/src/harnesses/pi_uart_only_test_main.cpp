/*
 * Minimal Pi<->arm UART bench harness -- see platformio.ini's
 * [env:pi-uart-only-test].
 *
 * Isolates the pi_uart link from every other arm-board subsystem: no
 * drivetrain_uart, no steppers, no ToF, no odometry, no dispatcher. This
 * board originates PiRequestPackets itself (nothing needs to arrive from the
 * drivetrain board), so it exercises the exact same pi_uart peripheral,
 * pins, and wire as production with everything else that normally runs on
 * the arm board during a mission removed.
 *
 * This is how UART_NUM_2 (the original PI_UART_LINK_CONFIG peripheral) was
 * found to reliably delay/lose RX by a full request cycle, and how
 * UART_NUM_0 (its replacement -- see uart_link_config.c) was confirmed clean
 * over 30+ cycles. Reusable for retesting the link on its own if the bug
 * ever resurfaces.
 */
#include <Arduino.h>
#include "driver/uart.h"

#include <robot_common/pi_action_packet.h>
#include <robot_common/uart_link.h>

#include "config/uart_link_config.h"

namespace {

constexpr uint32_t kRequestIntervalMs = 2000U;
constexpr uint32_t kResponseTimeoutMs = 3000U;

UartLink pi_uart = {};
uint8_t request_id = 0U;
bool waiting = false;
uint32_t last_send_ms = 0U;
uint32_t deadline_ms = 0U;

// Signed subtraction keeps deadline checks valid across millis() wraparound.
bool deadline_reached(uint32_t now_ms, uint32_t deadline) {
    return static_cast<int32_t>(now_ms - deadline) >= 0;
}

void send_request(uint32_t now_ms) {
    ++request_id;
    const PiRequestPacket request = {
        .request_id = request_id,
        .action = PI_ACTION_SCAN_TELETUBBIES,
        .parameter = 0.0f,
    };
    const esp_err_t error = pi_request_packet_send(&pi_uart, &request);
    last_send_ms = now_ms;
    deadline_ms = now_ms + kResponseTimeoutMs;
    waiting = true;
    if (error == ESP_OK) {
        Serial.printf("PIUART,SENT,id=%u,t=%lu\n", request_id,
                      static_cast<unsigned long>(now_ms));
    } else {
        Serial.printf("PIUART,SEND_FAILED,%s\n", esp_err_to_name(error));
    }
}

void print_link_stats(uint32_t now_ms) {
    size_t buffered_len = 0U;
    uart_get_buffered_data_len(pi_uart.config->uart_num, &buffered_len);
    Serial.printf(
        "PIUART,STATS,t=%lu,sent=%lu,received=%lu,checksum_errors=%lu,"
        "parse_errors=%lu,overwritten=%lu,buffered_now=%lu\n",
        static_cast<unsigned long>(now_ms),
        static_cast<unsigned long>(pi_uart.packets_sent),
        static_cast<unsigned long>(pi_uart.packets_received),
        static_cast<unsigned long>(pi_uart.checksum_errors),
        static_cast<unsigned long>(pi_uart.parse_errors),
        static_cast<unsigned long>(pi_uart.packets_overwritten),
        static_cast<unsigned long>(buffered_len));
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);

    const esp_err_t error = uart_link_init(&pi_uart, &PI_UART_LINK_CONFIG);
    if (error != ESP_OK) {
        Serial.printf("PIUART,FAULT,uart_init,%s\n", esp_err_to_name(error));
    }
    Serial.println("PIUART,READY,arm-pi-only");
    Serial.printf("PIUART,HELP,auto-requesting every %ums, timeout %ums\n",
                  static_cast<unsigned>(kRequestIntervalMs),
                  static_cast<unsigned>(kResponseTimeoutMs));
}

void loop() {
    const uint32_t now_ms = millis();

    if (!waiting && now_ms - last_send_ms >= kRequestIntervalMs) {
        send_request(now_ms);
    }

    const esp_err_t update_error = uart_link_update(&pi_uart);
    if (update_error != ESP_OK) {
        Serial.printf("PIUART,ERROR,uart_update,%s\n", esp_err_to_name(update_error));
    }

    if (waiting) {
        PacketFrame frame = {};
        if (uart_link_take_packet(&pi_uart, &frame) == ESP_OK) {
            PiReportPacket report = {};
            if (pi_report_packet_decode(&frame, &report) == ESP_OK &&
                report.request_id == request_id) {
                waiting = false;
                Serial.printf(
                    "PIUART,REPORT,id=%u,result=%u,target=%u,error=%.3f,"
                    "conf=%u,t=%lu,rtt_ms=%lu\n",
                    report.request_id, report.result, report.target_id,
                    report.horizontal_error, report.confidence_percent,
                    static_cast<unsigned long>(now_ms),
                    static_cast<unsigned long>(now_ms - last_send_ms));
            } else {
                // request_id here vs. the current request_id tells you
                // whether this is a stale reply from the previous cycle
                // (id == request_id - 1) or something structurally wrong.
                Serial.printf(
                    "PIUART,MISMATCH,t=%lu,type=%u,len=%u,payload=[",
                    static_cast<unsigned long>(now_ms), frame.message_type,
                    frame.payload_len);
                for (uint8_t i = 0U; i < frame.payload_len; ++i) {
                    Serial.printf("%02X ", frame.payload[i]);
                }
                Serial.println("]");
            }
        }

        if (waiting && deadline_reached(now_ms, deadline_ms)) {
            waiting = false;
            Serial.printf("PIUART,TIMEOUT,id=%u,t=%lu\n", request_id,
                          static_cast<unsigned long>(now_ms));
            print_link_stats(now_ms);
        }
    }

    delay(1);
}
