"""Implement the Raspberry Pi endpoint of the robot's framed task protocol.

This module mirrors the ESP32 command, status, heartbeat, and packet layouts.
PiTaskServer accepts only scan actions, detects stale/reset requesters, emits
start/cancel events for the scanner, and reports correlated terminal results.
"""

from __future__ import annotations

import secrets
import struct
import time
from dataclasses import dataclass

# Framed packet constants shared with robot-common/packet_protocol.h.
MAGIC = b"\xAA\x55"
VERSION = 0x01
PACKET_MAX_PAYLOAD_SIZE = 64

# Packet message identifiers; only task command/status/heartbeat are handled.
PACKET_TYPE_INVALID = 0
PACKET_TYPE_ODOMETRY = 1
PACKET_TYPE_RESERVED_COMMAND = 2
PACKET_TYPE_RESERVED_STATUS = 3
PACKET_TYPE_TASK_COMMAND = 4
PACKET_TYPE_TASK_STATUS = 5
PACKET_TYPE_HEARTBEAT = 6
PACKET_TYPE_MAX = 7

# Task command operations and endpoint identities.
TASK_COMMAND_START = 0
TASK_COMMAND_CANCEL = 1

TASK_ENDPOINT_TOP = 1
TASK_ENDPOINT_PI = 2

# The single physical task action implemented on the Raspberry Pi.
TASK_ACTION_SCAN_TELETUBBIES = 6

# Step lifecycle and failure values mirrored from task.h.
TASK_STEP_RUNNING = 1
TASK_STEP_SUCCEEDED = 2
TASK_STEP_CANCELLED = 3
TASK_STEP_FAILED = 4

TASK_FAILURE_NONE = 0
TASK_FAILURE_BUSY = 1
TASK_FAILURE_STEP_REJECTED = 3
TASK_FAILURE_LINK_TIMEOUT = 6
TASK_FAILURE_STALE_MESSAGE = 8
TASK_FAILURE_PROTOCOL = 9
TASK_FAILURE_EXECUTOR_UNAVAILABLE = 12
TASK_FAILURE_TARGET_NOT_FOUND = 13

# Fixed little-endian payload layouts shared with task_protocol.c.
COMMAND_FORMAT = "<IIIBBBBffffI"
STATUS_FORMAT = "<IIIIBB"
HEARTBEAT_FORMAT = "<BI"


def encode_frame(message_type: int, payload: bytes = b"") -> bytes:
    """Encode one validated message type and payload as a checksummed frame."""
    if not 0 < message_type < PACKET_TYPE_MAX:
        raise ValueError("invalid packet type")
    if len(payload) > PACKET_MAX_PAYLOAD_SIZE:
        raise ValueError("payload too large")
    body = bytes((VERSION, message_type, len(payload))) + payload
    checksum = 0
    for byte in body:
        checksum ^= byte
    return MAGIC + body + bytes((checksum,))


class PacketParser:
    """Incrementally reconstructs checksum-validated UART frames."""

    def __init__(self) -> None:
        """Create an incremental parser with an empty receive buffer."""
        self.buffer = bytearray()

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        """Consume serial bytes and return all complete, checksum-valid packets."""
        self.buffer.extend(data)
        packets: list[tuple[int, bytes]] = []
        while True:
            start = self.buffer.find(MAGIC)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b"\xAA" else b""
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 6:
                break
            version, message_type, payload_len = self.buffer[2:5]
            if (version != VERSION or not 0 < message_type < PACKET_TYPE_MAX or
                    payload_len > PACKET_MAX_PAYLOAD_SIZE):
                del self.buffer[0]
                continue
            frame_len = 6 + payload_len
            if len(self.buffer) < frame_len:
                break
            frame = bytes(self.buffer[:frame_len])
            checksum = 0
            for byte in frame[2:-1]:
                checksum ^= byte
            del self.buffer[:frame_len]
            if checksum == frame[-1]:
                packets.append((message_type, frame[5:-1]))
        return packets


@dataclass(frozen=True)
class TaskCommand:
    """Immutable command fields decoded from one ESP32 task payload."""

    requester_session_id: int  # Arm ESP32 boot-session identity.
    execution_id: int  # Coordinator task-run identity.
    command_id: int  # Monotonic command identity within the session.
    command_type: int  # Start or cancel operation.
    action: int  # Physical task action requested by the top dispatcher.
    step: int  # Zero-based coordinator workflow index.
    tape_direction: int  # Legacy tape wire field; unused by scan.
    tape_speed: float  # Legacy tape wire field; unused by scan.
    tape_distance: float  # Legacy tape wire field; unused by scan.
    amount: float  # Generic signed distance or angle; unused by scan.
    speed: float  # Generic linear or angular speed; unused by scan.
    settle_ms: int  # Generic settling time; unused by scan.


def decode_command(payload: bytes) -> TaskCommand | None:
    """Decode a command payload, returning None for invalid size or identity."""
    if len(payload) != struct.calcsize(COMMAND_FORMAT):
        return None
    values = struct.unpack(COMMAND_FORMAT, payload)
    command = TaskCommand(*values)
    if (command.requester_session_id == 0 or command.execution_id == 0 or
            command.command_id == 0 or
            command.command_type not in (TASK_COMMAND_START, TASK_COMMAND_CANCEL)):
        return None
    return command


def sequence_is_newer(candidate: int, reference: int) -> bool:
    """Compare wrapping unsigned 32-bit command sequence numbers."""
    difference = (candidate - reference) & 0xFFFFFFFF
    return difference != 0 and difference < 0x80000000


class PiTaskServer:
    """Serve scan commands idempotently without sequencing robot workflows."""

    def __init__(self, port: str, baud: int = 115200) -> None:
        """Open ``port`` at ``baud`` and initialize an idle Pi task session."""
        import serial

        self.serial = serial.Serial(port, baud, timeout=0)  # Nonblocking UART.
        self.parser = PacketParser()  # Incremental framed-packet decoder.
        self.session_id = secrets.randbits(32) or 1  # This Pi boot session.
        self.requester_session_id = 0  # Current arm ESP32 session.
        self.previous_requester_session_id = 0  # Session rejected as stale.
        self.last_receive = time.monotonic()  # Last accepted requester traffic.
        self.last_heartbeat = 0.0  # Last heartbeat transmit time.
        self.last_status = 0.0  # Last status transmit time.
        self.command: TaskCommand | None = None  # Current correlated command.
        self.link_timed_out = False  # Suppresses repeated timeout transitions.
        self.status = TASK_STEP_CANCELLED  # Latest command lifecycle state.
        self.failure = TASK_FAILURE_NONE  # Failure paired with ``status``.

    def _send(self, message_type: int, payload: bytes) -> None:
        """Frame and write one protocol payload to the arm ESP32."""
        self.serial.write(encode_frame(message_type, payload))

    def _send_heartbeat(self) -> None:
        """Advertise the Pi endpoint and its current boot-session identity."""
        self._send(PACKET_TYPE_HEARTBEAT,
                   struct.pack(HEARTBEAT_FORMAT, TASK_ENDPOINT_PI,
                               self.session_id))

    def _send_status(self, command: TaskCommand, status: int | None = None,
                     failure: int | None = None) -> None:
        """Send status correlated to ``command``, optionally overriding state."""
        self._send(
            PACKET_TYPE_TASK_STATUS,
            struct.pack(
                STATUS_FORMAT,
                command.requester_session_id,
                self.session_id,
                command.execution_id,
                command.command_id,
                self.status if status is None else status,
                self.failure if failure is None else failure,
            ),
        )
        self.last_status = time.monotonic()

    def complete(self, status: int, failure: int = TASK_FAILURE_NONE) -> None:
        """Finish the active command with a validated terminal status/failure."""
        if self.command is None or self.status != TASK_STEP_RUNNING:
            return
        if status not in (TASK_STEP_SUCCEEDED, TASK_STEP_CANCELLED,
                          TASK_STEP_FAILED):
            raise ValueError("completion must be terminal")
        if (status == TASK_STEP_FAILED) == (failure == TASK_FAILURE_NONE):
            raise ValueError("failure code does not match status")
        self.status = status
        self.failure = failure
        self._send_status(self.command)

    def _reset_requester(self, session_id: int,
                         remember_previous: bool = True) -> bool:
        """Adopt ``session_id`` and report whether running work was cancelled."""
        cancelled = self.command is not None and self.status == TASK_STEP_RUNNING
        if (remember_previous and self.requester_session_id and
                self.requester_session_id != session_id):
            self.previous_requester_session_id = self.requester_session_id
        self.requester_session_id = session_id
        self.command = None
        self.link_timed_out = False
        self.status = TASK_STEP_CANCELLED
        self.failure = TASK_FAILURE_NONE
        self.last_receive = time.monotonic()
        return cancelled

    def _handle_command(self, command: TaskCommand) -> str | None:
        """Validate command ordering and return a scanner start/cancel event."""
        if command.requester_session_id == self.previous_requester_session_id:
            self._send_status(command, TASK_STEP_FAILED,
                              TASK_FAILURE_STALE_MESSAGE)
            return None
        if command.requester_session_id != self.requester_session_id:
            reset_cancelled = self._reset_requester(
                command.requester_session_id)
            if reset_cancelled:
                return "cancel"
        self.last_receive = time.monotonic()
        self.link_timed_out = False

        if (self.command is not None and
                command.execution_id == self.command.execution_id and
                command.command_id == self.command.command_id):
            if command.command_type == TASK_COMMAND_CANCEL and self.status == TASK_STEP_RUNNING:
                self.status = TASK_STEP_CANCELLED
                self.failure = TASK_FAILURE_NONE
                self._send_status(command)
                return "cancel"
            self._send_status(command)
            return None

        if (self.command is not None and
                not sequence_is_newer(command.command_id,
                                      self.command.command_id)):
            self._send_status(command, TASK_STEP_FAILED,
                              TASK_FAILURE_STALE_MESSAGE)
            return None
        if command.command_type == TASK_COMMAND_CANCEL:
            self._send_status(command, TASK_STEP_FAILED,
                              TASK_FAILURE_STALE_MESSAGE)
            return None
        if self.command is not None and self.status == TASK_STEP_RUNNING:
            self._send_status(command, TASK_STEP_FAILED, TASK_FAILURE_BUSY)
            return None
        if command.action != TASK_ACTION_SCAN_TELETUBBIES:
            self._send_status(command, TASK_STEP_FAILED,
                              TASK_FAILURE_STEP_REJECTED)
            return None

        self.command = command
        self.status = TASK_STEP_RUNNING
        self.failure = TASK_FAILURE_NONE
        self._send_status(command)
        return "start"

    def update(self) -> list[str]:
        """Process UART traffic and timers, returning scanner control events."""
        now = time.monotonic()
        events: list[str] = []
        for message_type, payload in self.parser.feed(self.serial.read(256)):
            if message_type == PACKET_TYPE_HEARTBEAT and len(payload) == 5:
                sender, session_id = struct.unpack(HEARTBEAT_FORMAT, payload)
                if sender == TASK_ENDPOINT_TOP:
                    if session_id == self.previous_requester_session_id:
                        continue
                    if session_id != self.requester_session_id:
                        if self._reset_requester(session_id):
                            events.append("cancel")
                    else:
                        self.last_receive = now
                        self.link_timed_out = False
            elif message_type == PACKET_TYPE_TASK_COMMAND:
                command = decode_command(payload)
                if command is not None:
                    event = self._handle_command(command)
                    if event is not None:
                        events.append(event)

        if now - self.last_heartbeat >= 0.25:
            self._send_heartbeat()
            self.last_heartbeat = now
        if (self.requester_session_id and not self.link_timed_out and
                now - self.last_receive >= 2.0):
            if self.command is not None and self.status == TASK_STEP_RUNNING:
                self.status = TASK_STEP_FAILED
                self.failure = TASK_FAILURE_LINK_TIMEOUT
                events.append("cancel")
            self.link_timed_out = True
        if (self.command is not None and now - self.last_status >= 0.25):
            self._send_status(self.command)
        return events

    def close(self) -> None:
        """Close the serial transport owned by this task server."""
        self.serial.close()
