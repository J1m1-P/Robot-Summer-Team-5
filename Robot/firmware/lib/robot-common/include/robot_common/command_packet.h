/*
 * Defines commands issued by the drivetrain to arm-side action controllers.
 * The API sends, identifies, and decodes its fixed wire representation.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/uart_link.h>

#ifdef __cplusplus
extern "C" {
#endif

// Stores one opcode byte and one signed, x100-scaled value byte.
#define COMMAND_PACKET_PAYLOAD_SIZE 2U

// Identifies the action a command packet requests.
typedef enum {
    CMD_STOP = 0,
    CMD_TURN,
    CMD_SCAN,
    CMD_FOLLOW,
    CMD_FLASH,
    CMD_DONE,
    CMD_RESUME,   // continue an interrupted sweep/drive routine (reactive detector)


    // Tower Tasks
    CMD_TOWER_HOME,
    CMD_TOWER_Z_UP,
    CMD_TOWER_Z_DOWN,
    CMD_TOWER_X_LEFT,
    CMD_TOWER_X_RIGHT,
    CMD_TOWER_ROTATE_VERTICAL,
    CMD_TOWER_ROTATE_HORIZONTAL,
    CMD_TOWER_OPEN_CLAW,
    CMD_TOWER_CLOSE_CLAW,

    // Habitat Tasks
    CMD_HABITAT_HOME,
    CMD_HABITAT_Z_UP,
    CMD_HABITAT_Z_DOWN,
    CMD_HABITAT_X_LEFT,
    CMD_HABITAT_X_RIGHT,
    CMD_HABITAT_OPEN_CLAWS,
    CMD_HABITAT_CLOSE_CLAWS,
    CMD_HABITAT_OPEN_LEFT_CLAW,
    CMD_HABITAT_CLOSE_LEFT_CLAW,
    CMD_HABITAT_OPEN_RIGHT_CLAW,
    CMD_HABITAT_CLOSE_RIGHT_CLAW,

    // Raspberry Pi Tasks
    CMD_PI_SCAN_TELETUBBIES,

    CMD_MAX
} CommandOpcode;

// Represents one decoded command: an opcode plus its (optional) steering value.
typedef struct {
    CommandOpcode opcode;
    // TURN: meaning depends on which link this travels over -- the bridge
    // between them (esp32-arm/src/comms/pi_bridge.c) converts one to the
    // other, it isn't the same number on both hops:
    //   Pi -> arm:         normalized visual steering error, roughly -1.0..1.0.
    //   arm -> drivetrain:  commanded angular velocity in rad/s (positive CCW,
    //                       matching DrivetrainBodyVelocity.omega).
    // Tower/Habitat stepper commands: requested travel in units of 100 mm.
    // For example, 0.50 requests 50 mm and 0.30 requests 30 mm.
    // Pi commands: optional action parameter forwarded in the Pi request.
    // Unused (0) for all other opcodes.
    float value;
} CommandPacket;

// Validates, encodes, and sends a command packet over a UART link.
esp_err_t command_packet_send(UartLink *link, const CommandPacket *packet);

// Checks whether a generic packet frame has the command type and expected size.
bool command_packet_is(const PacketFrame *frame);

// Validates and decodes a command frame into a caller-provided structure.
esp_err_t command_packet_decode(const PacketFrame *frame, CommandPacket *packet_out);

#ifdef __cplusplus
}
#endif
