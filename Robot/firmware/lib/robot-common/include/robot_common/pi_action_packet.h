/*
 * Defines the request/report packets exchanged with the Raspberry Pi.
 * The arm ESP32 starts one Pi action, validates its reply, then relays the
 * report unchanged to the drivetrain for the robot sequence to interpret.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/uart_link.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PI_REQUEST_PACKET_PAYLOAD_SIZE 3U
#define PI_REPORT_PACKET_PAYLOAD_SIZE 7U

// Identifies the operation the Raspberry Pi should perform.
typedef enum {
    PI_ACTION_SCAN_TELETUBBIES = 0,
    PI_ACTION_MAX
} PiAction;

// Describes the outcome of a Pi action.
typedef enum {
    PI_RESULT_OK = 0,
    PI_RESULT_NOT_FOUND,
    PI_RESULT_TIMEOUT,
    PI_RESULT_CAMERA_FAULT,
    PI_RESULT_LINK_ERROR,
    PI_RESULT_INVALID_REQUEST,
    // Not a detection: horizontal_error is a commanded correction rotation
    // (e.g. undoing an earlier alignment turn to re-expose a second target),
    // not a fresh measurement. Apply it like PI_RESULT_OK's rotation, but
    // don't count it against the alignment attempt budget.
    PI_RESULT_REPOSITION,
    // Every target has been flashed. Not a detection -- there is nothing
    // left to search for, so the drivetrain should skip any remaining
    // search-checkpoint steps and move straight on.
    PI_RESULT_ALL_FOUND,
    PI_RESULT_MAX
} PiResultCode;

// Starts one Pi operation. The Pi must echo request_id in its report.
typedef struct {
    uint8_t request_id;
    PiAction action;
    float parameter;
} PiRequestPacket;

// Carries the useful Pi result through the arm ESP32 to the drivetrain.
// horizontal_error is normalized: -1 is left and +1 is right.
typedef struct {
    uint8_t request_id;
    PiAction action;
    PiResultCode result;
    uint8_t target_id;
    float horizontal_error;
    uint8_t confidence_percent;
} PiReportPacket;

esp_err_t pi_request_packet_send(
    UartLink *link,
    const PiRequestPacket *packet);
bool pi_request_packet_is(const PacketFrame *frame);
esp_err_t pi_request_packet_decode(
    const PacketFrame *frame,
    PiRequestPacket *packet_out);

esp_err_t pi_report_packet_send(
    UartLink *link,
    const PiReportPacket *packet);
bool pi_report_packet_is(const PacketFrame *frame);
esp_err_t pi_report_packet_decode(
    const PacketFrame *frame,
    PiReportPacket *packet_out);

#ifdef __cplusplus
}
#endif
