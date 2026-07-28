"""
uart_link.py — Python (detector) end of the ESP32 packet link
================================================================================
Mirrors the wire format in robot_common/uart_link.c so the ESP32 accepts packets
we send and so we can decode packets it sends. In the arm-controlled flow, the
Pi receives PI_REQUEST packets and answers with PI_REPORT packets.

TWO LAYERS:
  1. OUTER FRAME — fully defined by the C files, so this code is exact:
        [0xAA][0x55][VERSION=0x01][TYPE][LEN][PAYLOAD...][CHECKSUM]
        CHECKSUM = XOR(version, type, len, every payload byte)   (magic excluded)
  2. PAYLOADS — defined by the matching robot_common packet headers. The
     constants and scaling below mirror those headers, so both copies must be
     changed together.
"""

import struct


# ─────────────────────────────────────────────
# OUTER FRAME — verified against uart_link.c / packet_protocol.h. Safe to trust.
# ─────────────────────────────────────────────
MAGIC   = bytes([0xAA, 0x55])
VERSION = 0x01

# PacketMessageType, from packet_protocol.h
PACKET_TYPE_INVALID  = 0
PACKET_TYPE_ODOMETRY = 1
PACKET_TYPE_COMMAND  = 2
PACKET_TYPE_STATUS   = 3
PACKET_TYPE_PI_REQUEST = 4
PACKET_TYPE_PI_REPORT  = 5
PACKET_TYPE_MAX        = 6

PACKET_MAX_PAYLOAD_SIZE = 64


def encode_frame(msg_type, payload=b""):
    """
    Wrap a payload in the outer frame the ESP parser expects. The checksum XORs
    version, type, len, and the payload bytes — exactly calculate_checksum() in
    uart_link.c (the two magic bytes are excluded).
    """
    if len(payload) > PACKET_MAX_PAYLOAD_SIZE:
        raise ValueError("payload too big for one packet")
    body = bytes([VERSION, msg_type, len(payload)]) + payload
    checksum = 0
    for b in body:
        checksum ^= b
    return MAGIC + body + bytes([checksum])


# ─────────────────────────────────────────────
# COMMAND PAYLOAD — PROPOSAL. Must match the ESP firmware's decode. Coordinate!
# ─────────────────────────────────────────────
CMD_STOP   = 0
CMD_TURN   = 1   # value = steering error (-1..+1). A PIVOT IN PLACE — used by the
                 #   stop-and-pivot ALIGN in detector_reactive.py.
CMD_SCAN90 = 2   # start the initial 90° sweep (value unused)
CMD_FOLLOW = 3   # drive / tape-follow the course (value unused)
CMD_FLASH  = 4
CMD_DONE   = 5
CMD_RESUME = 6   # continue an interrupted sweep/drive routine (reactive detector)
CMD_TOWER_HOME = 7
CMD_TOWER_Z_UP = 8
CMD_TOWER_Z_DOWN = 9
CMD_TOWER_X_LEFT = 10
CMD_TOWER_X_RIGHT = 11
CMD_TOWER_ROTATE_VERTICAL = 12
CMD_TOWER_ROTATE_HORIZONTAL = 13
CMD_TOWER_OPEN_CLAW = 14
CMD_TOWER_CLOSE_CLAW = 15
CMD_HABITAT_HOME = 16
CMD_HABITAT_Z_UP = 17
CMD_HABITAT_Z_DOWN = 18
CMD_HABITAT_X_LEFT = 19
CMD_HABITAT_X_RIGHT = 20
CMD_HABITAT_OPEN_CLAWS = 21
CMD_HABITAT_CLOSE_CLAWS = 22
CMD_HABITAT_OPEN_LEFT_CLAW = 23
CMD_HABITAT_CLOSE_LEFT_CLAW = 24
CMD_HABITAT_OPEN_RIGHT_CLAW = 25
CMD_HABITAT_CLOSE_RIGHT_CLAW = 26
CMD_PI_SCAN_TELETUBBIES = 27

# PiAction and PiResultCode, from pi_action_packet.h
PI_ACTION_SCAN_TELETUBBIES = 0

PI_RESULT_OK = 0
PI_RESULT_NOT_FOUND = 1
PI_RESULT_TIMEOUT = 2
PI_RESULT_CAMERA_FAULT = 3
PI_RESULT_LINK_ERROR = 4
PI_RESULT_INVALID_REQUEST = 5
PI_RESULT_REPOSITION = 6   # not a detection: horizontal_error is a commanded
                           # correction rotation (see pi_action_packet.h)
PI_RESULT_ALL_FOUND = 7    # not a detection: every target has been flashed,
                           # nothing left to search for (see pi_action_packet.h)
PI_RESULT_MAX = 8

# ─────────────────────────────────────────────
# STATUS PAYLOAD — mirrors robot_common/status_packet.h's StatusCode.
# ─────────────────────────────────────────────
STATUS_SCAN_DONE    = 0   # legacy: ESP finished its scan sweep (SCAN/FOLLOW detectors)
STATUS_LOOK_START   = 1   # the ESP opened its look-window (reactive detector: begin looking)
STATUS_LOOK_END     = 2   # the ESP closed its look-window (sweep done, nothing found)
STATUS_ROUTINE_DONE = 3   # the ESP ran out of sweeps/routine with targets still unfound
STATUS_FAULT        = 4   # something went wrong; detail byte holds a fault code
STATUS_ACTION_COMPLETE = 5

STATUS_DETAIL_NONE = 0
STATUS_DETAIL_TOWER_Z_RAISED = 1
STATUS_DETAIL_TOWER_Z_LOWERED = 2
STATUS_DETAIL_TOWER_X_LEFT = 3
STATUS_DETAIL_TOWER_X_RIGHT = 4
STATUS_DETAIL_TOWER_HOME = 5
STATUS_DETAIL_TOWER_VERTICAL = 6
STATUS_DETAIL_TOWER_HORIZONTAL = 7
STATUS_DETAIL_TOWER_CLAW_OPEN = 8
STATUS_DETAIL_TOWER_CLAW_CLOSED = 9
STATUS_DETAIL_HABITAT_HOME = 10
STATUS_DETAIL_HABITAT_Z_RAISED = 11
STATUS_DETAIL_HABITAT_Z_LOWERED = 12
STATUS_DETAIL_HABITAT_X_LEFT = 13
STATUS_DETAIL_HABITAT_X_RIGHT = 14
STATUS_DETAIL_HABITAT_CLAWS_OPEN = 15
STATUS_DETAIL_HABITAT_CLAWS_CLOSED = 16
STATUS_DETAIL_HABITAT_LEFT_CLAW_OPEN = 17
STATUS_DETAIL_HABITAT_LEFT_CLAW_CLOSED = 18
STATUS_DETAIL_HABITAT_RIGHT_CLAW_OPEN = 19
STATUS_DETAIL_HABITAT_RIGHT_CLAW_CLOSED = 20


def encode_command(opcode, value=0.0):
    """
    Build a COMMAND packet: payload = [opcode, signed value byte]. `value` is a
    float ~-1..+1; we send round(value*100) as a signed int8 (-100..100). The
    ESP recovers it as (int8_t)payload[1] / 100.0f. Commands without a value
    just send 0. Tower/Habitat stepper values are distances in units of
    100 mm, so value=0.50 requests 50 mm.
    """
    scaled = max(-127, min(127, int(round(value * 100))))
    payload = struct.pack("<Bb", opcode, scaled)   # B = opcode (uint8), b = value (int8)
    return encode_frame(PACKET_TYPE_COMMAND, payload)


def decode_pi_request(payload):
    """Return (request_id, action, parameter) for a valid Pi request."""
    if len(payload) != 3:
        raise ValueError("invalid Pi request length")
    request_id, action, raw_parameter = struct.unpack("<BBb", payload)
    if action != PI_ACTION_SCAN_TELETUBBIES:
        raise ValueError("invalid Pi action")
    return request_id, action, raw_parameter / 100.0


def encode_pi_report(request_id, action, result, target_id=0,
                     horizontal_error=0.0, confidence_percent=0):
    """Encode the report the arm ESP32 will relay to the drivetrain."""
    if action != PI_ACTION_SCAN_TELETUBBIES:
        raise ValueError("invalid Pi action")
    if result < PI_RESULT_OK or result >= PI_RESULT_MAX:
        raise ValueError("invalid Pi result")
    confidence_percent = max(0, min(100, int(confidence_percent)))
    error_x1000 = int(round(max(-1.0, min(1.0, horizontal_error)) * 1000))
    payload = struct.pack(
        "<BBBBhB",
        request_id & 0xFF,
        action,
        result,
        target_id & 0xFF,
        error_x1000,
        confidence_percent,
    )
    return encode_frame(PACKET_TYPE_PI_REPORT, payload)


# ─────────────────────────────────────────────
# RECEIVE — reassemble packets the ESP sends back
# ─────────────────────────────────────────────
class PacketParser:
    """
    Rebuilds framed packets from a byte stream — the Python mirror of the state
    machine in uart_link.c. Feed it bytes one at a time; feed() returns a
    completed (msg_type, payload) when a full, checksum-valid packet arrives,
    and None otherwise. Bad version/type/length/checksum just resets the parser.
    """
    _MAGIC0, _MAGIC1, _VERSION, _TYPE, _LEN, _PAYLOAD, _CHECKSUM = range(7)

    def __init__(self):
        self._reset()

    def _reset(self):
        self.state = self._MAGIC0
        self.msg_type = 0
        self.payload_len = 0
        self.checksum = 0
        self.payload = bytearray()

    def feed(self, byte):
        s = self.state
        if s == self._MAGIC0:
            if byte == 0xAA:
                self.state = self._MAGIC1
        elif s == self._MAGIC1:
            if byte == 0x55:
                self.state = self._VERSION
            elif byte != 0xAA:
                self._reset()
        elif s == self._VERSION:
            if byte != VERSION:
                self._reset()
            else:
                self.checksum = byte           # checksum starts from the version byte
                self.state = self._TYPE
        elif s == self._TYPE:
            if byte <= PACKET_TYPE_INVALID or byte >= PACKET_TYPE_MAX:
                self._reset()
            else:
                self.msg_type = byte
                self.checksum ^= byte
                self.state = self._LEN
        elif s == self._LEN:
            if byte > PACKET_MAX_PAYLOAD_SIZE:
                self._reset()
            else:
                self.payload_len = byte
                self.payload = bytearray()
                self.checksum ^= byte
                self.state = self._CHECKSUM if byte == 0 else self._PAYLOAD
        elif s == self._PAYLOAD:
            self.payload.append(byte)
            self.checksum ^= byte
            if len(self.payload) >= self.payload_len:
                self.state = self._CHECKSUM
        elif s == self._CHECKSUM:
            complete = (self.msg_type, bytes(self.payload)) if byte == self.checksum else None
            self._reset()
            return complete
        return None


# ─────────────────────────────────────────────
# The link object your detector talks to
# ─────────────────────────────────────────────
class RobotLink:
    """Sends commands/reports and decodes packets over the ESP32 serial link."""

    def __init__(self, port, baud=115200, rx_chunk=256):
        import serial   # imported lazily so the encoders work without pyserial installed
        # `port`: "COM5" on Windows, "/dev/ttyUSB0" or "/dev/serial0" on the Pi.
        # `baud`: MUST equal UartLinkConfig.baud_rate on the ESP side.
        # timeout=0 -> non-blocking reads/writes, so the vision loop never stalls.
        self.ser = serial.Serial(port, baud, timeout=0)
        self.parser = PacketParser()
        self.rx_chunk = rx_chunk

    # --- sending ---
    def send_command(self, opcode, value=0.0):
        self._write(encode_command(opcode, value))

    def send_pi_report(self, request_id, action, result, target_id=0,
                       horizontal_error=0.0, confidence_percent=0):
        self._write(encode_pi_report(
            request_id, action, result, target_id,
            horizontal_error, confidence_percent))

    def _write(self, frame):
        # A disconnected/flaky USB-serial link should drop this one packet,
        # not crash the caller's control loop.
        try:
            self.ser.write(frame)
        except OSError as e:
            print(f"[uart_link] write failed: {e}")

    def stop(self):          self.send_command(CMD_STOP)
    def turn(self, error):   self.send_command(CMD_TURN, error)
    def scan90(self):        self.send_command(CMD_SCAN90)
    def follow(self):        self.send_command(CMD_FOLLOW)
    def flash(self):         self.send_command(CMD_FLASH)
    def done(self):          self.send_command(CMD_DONE)
    def resume(self):        self.send_command(CMD_RESUME)
    def tower_z_up(self, distance_mm=100):
        self.send_command(CMD_TOWER_Z_UP, distance_mm / 100.0)
    def tower_z_down(self, distance_mm=100):
        self.send_command(CMD_TOWER_Z_DOWN, distance_mm / 100.0)
    def tower_rotate_vertical(self):
        self.send_command(CMD_TOWER_ROTATE_VERTICAL)
    def tower_rotate_horizontal(self):
        self.send_command(CMD_TOWER_ROTATE_HORIZONTAL)
    def tower_open_claw(self):
        self.send_command(CMD_TOWER_OPEN_CLAW)
    def tower_close_claw(self):
        self.send_command(CMD_TOWER_CLOSE_CLAW)

    # --- receiving ---
    def poll(self):
        """
        Read whatever bytes have arrived and return a list of completed
        (msg_type, payload) packets. Non-blocking — call it once per loop.
        Returns an empty list (rather than raising) if the link drops.
        """
        try:
            data = self.ser.read(self.rx_chunk)
        except OSError as e:
            print(f"[uart_link] read failed: {e}")
            return []

        packets = []
        for b in data:
            pkt = self.parser.feed(b)
            if pkt is not None:
                packets.append(pkt)
        return packets

    def close(self):
        self.ser.close()


def _feed_all(parser, frame):
    """Feed a whole frame through a parser and return the decoded packet."""
    out = None
    for b in frame:
        out = parser.feed(b) or out
    return out


# Self-test of the ENCODER + PARSER only (no hardware needed): python uart_link.py
if __name__ == "__main__":
    for name, pkt in [
        ("STOP", encode_command(CMD_STOP)),
        ("TURN -0.5", encode_command(CMD_TURN, -0.5)),
        ("PI_SCAN_TELETUBBIES", encode_command(CMD_PI_SCAN_TELETUBBIES)),
        ("TOWER_Z_UP 0.5", encode_command(CMD_TOWER_Z_UP, 0.5)),
    ]:
        print(f"{name:22s}:", pkt.hex(" "))

    # COMMAND round-trip.
    out = _feed_all(PacketParser(), encode_command(CMD_TURN, -0.5))
    assert out is not None and out[0] == PACKET_TYPE_COMMAND
    print("COMMAND round-trip:", out)

    # PI_REQUEST round-trip (the live path: arm ESP -> Pi, decoded by
    # decode_pi_request()).
    request = struct.pack("<BBb", 7, PI_ACTION_SCAN_TELETUBBIES, 0)
    out = _feed_all(PacketParser(), encode_frame(PACKET_TYPE_PI_REQUEST, request))
    assert out is not None and out[0] == PACKET_TYPE_PI_REQUEST
    decoded = decode_pi_request(out[1])
    assert decoded == (7, PI_ACTION_SCAN_TELETUBBIES, 0.0), decoded
    print("PI_REQUEST round-trip:", decoded)

    # PI_REPORT round-trip (the live path: Pi -> arm ESP, built by
    # encode_pi_report()/send_pi_report()).
    out = _feed_all(PacketParser(), encode_pi_report(
        7, PI_ACTION_SCAN_TELETUBBIES, PI_RESULT_OK,
        target_id=2, horizontal_error=-0.25, confidence_percent=80))
    assert out is not None and out[0] == PACKET_TYPE_PI_REPORT
    print("PI_REPORT round-trip:", out)

    print("uart_link self-test OK")
