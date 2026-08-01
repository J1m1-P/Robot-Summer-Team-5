"""
uart_echo_test.py — Pi<->arm UART round-trip test, no camera/YOLO involved
================================================================================
Isolates whether the flaky request_id / long-delay behavior seen from
tubby_detector.py is a UART/link problem or something in the camera+YOLO
pipeline. This script does the absolute minimum: as soon as a PI_REQUEST
arrives, it immediately answers with a fixed dummy PI_RESULT_NOT_FOUND report
and logs timestamps for both. No camera, no model, no Flask, no extra threads
-- just uart_link's request/report round trip by itself.

Run this INSTEAD of tubby_detector.py (both open the same serial port -- never
run both at once). Trigger scans from the drivetrain's pi-scan-test harness as
usual and compare what shows up here against the arm's own monitor:
  - If replies now show up fast and in order here, the link itself is fine and
    the problem lives somewhere in tubby_detector.py's camera/YOLO/Flask code.
  - If the same delay/out-of-order behavior still happens with this script
    (which never touches a camera), that points at the UART link/wiring/ESP
    side instead.

Ctrl-C to stop.
"""
import time

from uart_link import (
    RobotLink,
    PACKET_TYPE_PI_REQUEST,
    decode_pi_request,
    PI_ACTION_SCAN_TELETUBBIES,
    PI_RESULT_NOT_FOUND,
)

SERIAL_PORT = "/dev/serial0"   # ADJUST: must match tubby_detector.py / the arm's PI_UART_LINK_CONFIG
SERIAL_BAUD = 115200           # ADJUST: must match PI_UART_LINK_CONFIG on the ESP

link = RobotLink(SERIAL_PORT, SERIAL_BAUD)

print(f"[uart_echo_test] listening on {SERIAL_PORT} @ {SERIAL_BAUD} -- Ctrl-C to stop")

try:
    while True:
        for msg_type, payload in link.poll():
            if msg_type != PACKET_TYPE_PI_REQUEST:
                print(f"[uart_echo_test] unexpected packet type={msg_type}")
                continue
            try:
                request_id, action, parameter = decode_pi_request(payload)
            except ValueError as e:
                print(f"[uart_echo_test] bad request: {e}")
                continue
            if action != PI_ACTION_SCAN_TELETUBBIES:
                continue

            t_recv = time.monotonic()
            print(f"[uart_echo_test] REQUEST id={request_id} param={parameter:+.3f} "
                  f"t={t_recv:.3f}")

            link.send_pi_report(
                request_id, PI_ACTION_SCAN_TELETUBBIES, PI_RESULT_NOT_FOUND,
                target_id=0, horizontal_error=0.0, confidence_percent=0)

            t_sent = time.monotonic()
            print(f"[uart_echo_test] REPLY   id={request_id} t={t_sent:.3f} "
                  f"(handled in {(t_sent - t_recv) * 1000:.1f} ms)")

        time.sleep(0.01)
except KeyboardInterrupt:
    print("\n[uart_echo_test] stopped")
finally:
    link.close()
