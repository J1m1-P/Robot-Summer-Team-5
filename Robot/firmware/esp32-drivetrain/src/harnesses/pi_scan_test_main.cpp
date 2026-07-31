/*
 * Bench-only diagnostic for the Pi <-> arm <-> drivetrain teletubby-scan UART
 * chain. Sends one CMD_PI_SCAN_TELETUBBIES to the arm ESP on demand and
 * prints whatever PiReportPacket comes back. No drivetrain motors, pose
 * tracking, or robot_sequence_controller involved -- just the arm_uart link,
 * so this exercises the same wire path robot_sequence_controller.c uses for
 * ROBOT_STEP_PI_ALIGN without needing the robot to actually drive anywhere.
 *
 * Serial commands (type + Enter in the monitor):
 *   scan      send one CMD_PI_SCAN_TELETUBBIES
 *   identify  print a ready line, for scripted use
 */
#include <Arduino.h>

#include <robot_common/command_packet.h>
#include <robot_common/pi_action_packet.h>
#include <robot_common/uart_link.h>

#include "config/communication/uart_link_config.h"

namespace {

UartLink arm_uart = {};
String command;

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

void send_scan_request() {
    const CommandPacket command_packet = {
        .opcode = CMD_PI_SCAN_TELETUBBIES,
        .value = 0.0f,
    };
    const esp_err_t error = command_packet_send(&arm_uart, &command_packet);
    if (error == ESP_OK) {
        Serial.println("PISCAN,SENT");
    } else {
        Serial.printf("PISCAN,SEND_FAILED,%s\n", esp_err_to_name(error));
    }
}

void print_report(const PiReportPacket &report) {
    Serial.printf(
        "PISCAN,REPORT,request=%u,result=%s,target=%u,error=%.3f,conf=%u\n",
        report.request_id, result_name(report.result), report.target_id,
        report.horizontal_error, report.confidence_percent);
}

void handle_command(const String &line) {
    if (line == "scan") {
        send_scan_request();
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
    Serial.println(
        "PISCAN,HELP,type 'scan' + enter to send one CMD_PI_SCAN_TELETUBBIES");
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
        } else {
            Serial.printf("PISCAN,UNEXPECTED_PACKET,type=%u,len=%u\n",
                          frame.message_type, frame.payload_len);
        }
    }

    delay(1);
}
