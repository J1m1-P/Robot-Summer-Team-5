"""
uart_push_test.py — Pi -> arm one-way UART push test, no request/reply loop
================================================================================
Companion to pi_uart_rx_only_test_main.cpp. Unlike uart_echo_test.py (which
only replies when a PI_REQUEST arrives), this script pushes a dummy
PI_REPORT on its own fixed timer, unprompted -- the ESP side never transmits
anything on pi_uart at all.

Why: the round-trip test showed every reply invisible to the ESP until its
own next TX, no matter the request interval. That's equally consistent with
(a) the ESP's TX somehow servicing its own stuck RX, or (b) the Pi's write
not physically leaving the wire promptly despite flush() succeeding. Since
this script's writes don't depend on anything arriving from the ESP first,
if the ESP still only sees these pushes late/stuck, the ESP's TX activity
(which isn't happening here) cannot be the cause -- pointing at the Pi's
transmit side or the physical wire instead.

Run this on the Pi, pi_uart_rx_only_test_main.cpp on the arm board. Compare
this script's own send timestamps against the arm monitor's PIUARTRX,GOT
timestamps.

Ctrl-C to stop.
"""
import time

from uart_link import RobotLink, PI_ACTION_SCAN_TELETUBBIES, PI_RESULT_NOT_FOUND

SERIAL_PORT = "/dev/serial0"   # ADJUST: must match the arm's PI_UART_LINK_CONFIG
SERIAL_BAUD = 115200           # ADJUST: must match PI_UART_LINK_CONFIG on the ESP
PUSH_PERIOD_S = 2.0

link = RobotLink(SERIAL_PORT, SERIAL_BAUD)

print(f"[uart_push_test] pushing on {SERIAL_PORT} @ {SERIAL_BAUD} "
      f"every {PUSH_PERIOD_S}s -- Ctrl-C to stop")

request_id = 0
try:
    while True:
        request_id = (request_id + 1) & 0xFF
        t_send = time.monotonic()
        link.send_pi_report(
            request_id, PI_ACTION_SCAN_TELETUBBIES, PI_RESULT_NOT_FOUND,
            target_id=0, horizontal_error=0.0, confidence_percent=0)
        print(f"[uart_push_test] PUSH id={request_id} t={t_send:.3f}")
        time.sleep(PUSH_PERIOD_S)
except KeyboardInterrupt:
    print("\n[uart_push_test] stopped")
finally:
    link.close()
