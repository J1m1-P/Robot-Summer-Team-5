/*
 * Bench-only diagnostic for the Pi <-> arm <-> drivetrain teletubby-scan UART
 * chain. Sends one CMD_PI_SCAN_TELETUBBIES to the arm ESP on demand and
 * prints whatever PiReportPacket comes back. No drivetrain motors, pose
 * tracking, or robot_sequence_controller involved -- just the arm_uart link,
 * so this exercises the same wire path robot_sequence_controller.c uses for
 * ROBOT_STEP_PI_ALIGN without needing the robot to actually drive anywhere.
 *
 * Auto-fires one CMD_PI_SCAN_TELETUBBIES every kAutoScanIntervalMs so you
 * don't have to type into this monitor to keep scans coming -- useful when
 * you're watching this board's monitor side by side with the arm ESP's own,
 * looking for where a report goes missing. Waits for the in-flight request
 * to finish (report received, or kAutoScanSafetyMs elapses with nothing
 * back at all) before firing the next one, so it never piles up requests
 * the arm would just reject as busy.
 *
 * Serial commands (type + Enter in the monitor):
 *   scan      send one CMD_PI_SCAN_TELETUBBIES immediately
 *   identify  print a ready line, for scripted use
 */
#include <Arduino.h>

#include <robot_common/command_packet.h>
#include <robot_common/odometry_packet.h>
#include <robot_common/pi_action_packet.h>
#include <robot_common/status_packet.h>
#include <robot_common/uart_link.h>

#include "config/communication/uart_link_config.h"

namespace {

// Gap between auto-fired scans once the previous one has finished.
constexpr uint32_t kAutoScanIntervalMs = 5000U;
// Safety fallback if a report never comes back at all, so a single
// fully-lost report can't stall the auto loop forever. Must stay above the
// arm's own PiActionController timeout (see pi_action_controller.cpp) or
// this harness will re-fire and get rejected as "busy" while the arm is
// still waiting -- kept 2s above it, currently TEMPORARILY raised to 25s
// for bench debugging.
constexpr uint32_t kAutoScanSafetyMs = 27000U;

UartLink arm_uart = {};
String command;
bool waiting_for_report = false;
uint32_t last_send_ms = 0U;
uint32_t scans_sent = 0U;

const char *result_name(PiResultCode result) {
    switch (result) {
        case PI_RESULT_OK: return "OK";
        case PI_RESULT_NOT_FOUND: return "NOT_FOUND";
        case PI_RESULT_TIMEOUT: return "TIMEOUT";
        case PI_RESULT_CAMERA_FAULT: return "CAMERA_FAULT";
        case PI_RESULT_LINK_ERROR: return "LINK_ERROR";
        case PI_RESULT_INVALID_REQUEST: return "INVALID_REQUEST";
        case PI_RESULT_REPOSITION: return "REPOSITION";
        case PI_RESULT_ALL_FOUND: return "ALL_FOUND";
        default: return "UNKNOWN";
    }
}

void send_scan_request(uint32_t now_ms) {
    const CommandPacket command_packet = {
        .opcode = CMD_PI_SCAN_TELETUBBIES,
        .value = 0.0f,
    };
    const esp_err_t error = command_packet_send(&arm_uart, &command_packet);
    last_send_ms = now_ms;
    waiting_for_report = true;
    if (error == ESP_OK) {
        ++scans_sent;
        Serial.printf("PISCAN,SENT,n=%u\n", scans_sent);
    } else {
        Serial.printf("PISCAN,SEND_FAILED,%s\n", esp_err_to_name(error));
    }
}

void print_report(const PiReportPacket &report) {
    waiting_for_report = false;
    Serial.printf(
        "PISCAN,REPORT,request=%u,result=%s,target=%u,error=%.3f,conf=%u\n",
        report.request_id, result_name(report.result), report.target_id,
        report.horizontal_error, report.confidence_percent);
}

void handle_command(const String &line) {
    if (line == "scan") {
        send_scan_request(millis());
    } else if (line == "identify") {
        Serial.println("PISCAN,READY,drivetrain");
    } else {
        Serial.printf("PISCAN,ERROR,unknown_command,%s\n", line.c_str());
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(300);

    const esp_err_t error =
        uart_link_init(&arm_uart, &TOP_ESP_UART_LINK_CONFIG);
    if (error != ESP_OK) {
        Serial.printf("PISCAN,FAULT,uart_init,%s\n", esp_err_to_name(error));
    }
    Serial.println("PISCAN,READY,drivetrain");
    Serial.printf(
        "PISCAN,HELP,auto-scanning every %ums; type 'scan' + enter for an "
        "extra one on demand\n",
        kAutoScanIntervalMs);
}

void loop() {
    while (Serial.available()) {
        const char value = static_cast<char>(Serial.read());
        Serial.write(value);   // local echo -- the monitor doesn't echo on its own
        if (value == '\n' || value == '\r') {
            command.trim();
            if (!command.isEmpty()) handle_command(command);
            command = "";
        } else if (command.length() < 32U) {
            command += value;
        }
    }

    const uint32_t now_ms = millis();
    const uint32_t elapsed_ms = now_ms - last_send_ms;
    const bool due = waiting_for_report ? elapsed_ms >= kAutoScanSafetyMs
                                         : elapsed_ms >= kAutoScanIntervalMs;
    if (due) {
        if (waiting_for_report) Serial.println("PISCAN,AUTO,no report -- retrying");
        send_scan_request(now_ms);
    }

    const esp_err_t update_error = uart_link_update(&arm_uart);
    if (update_error != ESP_OK) {
        Serial.printf("PISCAN,ERROR,uart_update,%s\n",
                      esp_err_to_name(update_error));
    }

    while (uart_link_has_packet(&arm_uart)) {
        PacketFrame frame = {};
        if (uart_link_take_packet(&arm_uart, &frame) != ESP_OK) break;
        if (pi_report_packet_is(&frame)) {
            PiReportPacket report = {};
            if (pi_report_packet_decode(&frame, &report) == ESP_OK) {
                print_report(report);
            } else {
                Serial.println("PISCAN,ERROR,decode_failed");
            }
        } else if (odometry_packet_is(&frame)) {
            // The arm's regular firmware streams these continuously to
            // whatever's on the other end of arm_uart, scan or no scan --
            // expected noise on this link, not worth printing.
            continue;
        } else if (status_packet_is(&frame)) {
            StatusPacket status = {};
            if (status_packet_decode(&frame, &status) == ESP_OK &&
                status.code == STATUS_ACTION_COMPLETE &&
                status.detail == STATUS_DETAIL_NONE) {
                // The arm's "ready" heartbeat -- repeats every 20ms until it
                // accepts its first command (see
                // arm_action_dispatcher.cpp's readiness_pending). Expected,
                // stops for good once the first "scan" is sent.
                continue;
            }
            Serial.printf("PISCAN,ARM_STATUS,code=%u,detail=%u\n",
                          status.code, status.detail);
        } else {
            Serial.printf("PISCAN,UNEXPECTED_PACKET,type=%u,len=%u\n",
                          frame.message_type, frame.payload_len);
        }
    }

    delay(1);
}
