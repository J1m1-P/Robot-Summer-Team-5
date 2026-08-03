/*
 * Defines the status packet the ESP sends back to the Pi to report robot state.
 * The API sends, identifies, and decodes its fixed wire representation.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/command_packet.h>
#include <robot_common/uart_link.h>

#ifdef __cplusplus
extern "C" {
#endif

// Stores one status code byte and one detail byte.
#define STATUS_PACKET_PAYLOAD_SIZE 2U

// Identifies what a status packet reports. Add new codes here as the ESP
// gains more things worth telling the Pi.
typedef enum {
    STATUS_SCAN_DONE = 0,   // legacy: ESP finished its scan sweep (SCAN/FOLLOW detectors)
    STATUS_LOOK_START,      // the ESP opened its look-window (reactive detector: begin looking)
    STATUS_LOOK_END,        // the ESP closed its look-window (sweep done, nothing found)
    STATUS_ROUTINE_DONE,    // the ESP ran out of sweeps/routine with targets still unfound
    STATUS_FAULT,           // something went wrong; `detail` holds a fault code
    STATUS_ACTION_COMPLETE, // arm ready when detail is NONE; otherwise an action completed
    STATUS_LOCATOR_CONTACT, // the extended Tower locator microswitch was pressed
    STATUS_SOLAR_PANEL_CONTACT, // the solar-panel microswitch was pressed
    STATUS_MAX
} StatusCode;

// Detail values used with STATUS_ACTION_COMPLETE by the simple system demo.
typedef enum {
    STATUS_DETAIL_NONE = 0,
    STATUS_DETAIL_TOWER_Z_RAISED,
    STATUS_DETAIL_TOWER_Z_LOWERED,
    STATUS_DETAIL_TOWER_X_LEFT,
    STATUS_DETAIL_TOWER_X_RIGHT,
    STATUS_DETAIL_TOWER_HOME,
    STATUS_DETAIL_TOWER_VERTICAL,
    STATUS_DETAIL_TOWER_HORIZONTAL,
    STATUS_DETAIL_TOWER_CLAW_OPEN,
    STATUS_DETAIL_TOWER_CLAW_CLOSED,
    STATUS_DETAIL_HABITAT_HOME,
    STATUS_DETAIL_HABITAT_Z_RAISED,
    STATUS_DETAIL_HABITAT_Z_LOWERED,
    STATUS_DETAIL_HABITAT_X_LEFT,
    STATUS_DETAIL_HABITAT_X_RIGHT,
    STATUS_DETAIL_HABITAT_CLAWS_OPEN,
    STATUS_DETAIL_HABITAT_CLAWS_CLOSED,
    STATUS_DETAIL_HABITAT_LEFT_CLAW_OPEN,
    STATUS_DETAIL_HABITAT_LEFT_CLAW_CLOSED,
    STATUS_DETAIL_HABITAT_RIGHT_CLAW_OPEN,
    STATUS_DETAIL_HABITAT_RIGHT_CLAW_CLOSED,
    STATUS_DETAIL_TOWER_Z_MOVED,
    STATUS_DETAIL_TOWER_X_MOVED,
    STATUS_DETAIL_TOWER_ALL_CLAWS_OPEN,
    STATUS_DETAIL_TOWER_ALL_CLAWS_CLOSED,
    STATUS_DETAIL_TOWER_LEFT_CLAW_OPEN,
    STATUS_DETAIL_TOWER_LEFT_CLAW_CLOSED,
    STATUS_DETAIL_TOWER_MIDDLE_CLAW_OPEN,
    STATUS_DETAIL_TOWER_MIDDLE_CLAW_CLOSED,
    STATUS_DETAIL_TOWER_RIGHT_CLAW_OPEN,
    STATUS_DETAIL_TOWER_RIGHT_CLAW_CLOSED,
    STATUS_DETAIL_TOWER_LOCATOR_EXTENDED,
    STATUS_DETAIL_TOWER_LOCATOR_RETRACTED,
    STATUS_DETAIL_HABITAT_Z_MOVED,
    STATUS_DETAIL_HABITAT_X_MOVED,
    STATUS_DETAIL_HABITAT_LEFT_CLAW_SEMI_CLOSED,
    STATUS_DETAIL_HABITAT_RIGHT_CLAW_SEMI_CLOSED,
} ActionStatusDetail;

// Maps any arm command to its command-specific completion detail.
ActionStatusDetail arm_action_status_detail(CommandOpcode command);

// Represents one decoded status: a code plus an optional detail byte.
typedef struct {
    StatusCode code;
    uint8_t detail;   // meaning depends on code; 0 when unused.
} StatusPacket;

// Validates, encodes, and sends a status packet over a UART link.
esp_err_t status_packet_send(UartLink *link, const StatusPacket *packet);

// Checks whether a generic packet frame has the status type and expected size.
bool status_packet_is(const PacketFrame *frame);

// Validates and decodes a status frame into a caller-provided structure.
esp_err_t status_packet_decode(const PacketFrame *frame, StatusPacket *packet_out);

#ifdef __cplusplus
}
#endif
