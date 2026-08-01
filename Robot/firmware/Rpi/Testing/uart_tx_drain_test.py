"""
uart_tx_drain_test.py — checks whether flush()/tcdrain() lies about TX completion
================================================================================
The RX-only ESP harness proved the ESP's receive path is clean on its own: with
the ESP never transmitting, unsolicited pushes from uart_push_test.py arrive
right on schedule, forever, no per-cycle lag. That rules out the ESP side for
the round-trip test's persistent "reply only shows up one cycle later" bug --
so the remaining suspect is whether the Pi's own write actually leaves the pin
when self.ser.flush() (termios tcdrain()) returns, or whether it's still sitting
in a kernel-level queue at that point.

This script writes a dummy frame, calls flush() same as uart_link.py does, and
then polls TIOCOUTQ (bytes still queued for transmission on this fd, per Linux
tty ioctl) every ~1ms until it hits zero or a 3s timeout. If flush() is honest,
TIOCOUTQ should already read 0 immediately after it returns. If it's still
nonzero for a while after flush() returns, that's a real, reproducible kernel/
driver-level finding: physical transmission is delayed independent of anything
the application does.

No ESP needed to run this -- it only inspects the Pi's own kernel-reported
queue depth. Ctrl-C to stop.
"""
import fcntl
import struct
import termios
import time

from uart_link import encode_pi_report, PI_ACTION_SCAN_TELETUBBIES, PI_RESULT_NOT_FOUND

SERIAL_PORT = "/dev/serial0"   # ADJUST: must match the arm's PI_UART_LINK_CONFIG
SERIAL_BAUD = 115200
PUSH_PERIOD_S = 2.0
POLL_TIMEOUT_S = 3.0


def outq_bytes(fd):
    """Bytes still sitting in this fd's kernel-level TX queue (Linux TIOCOUTQ)."""
    buf = fcntl.ioctl(fd, termios.TIOCOUTQ, struct.pack("I", 0))
    return struct.unpack("I", buf)[0]


def main():
    import serial
    ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0)
    fd = ser.fileno()

    print(f"[uart_tx_drain_test] writing to {SERIAL_PORT} @ {SERIAL_BAUD} "
          f"every {PUSH_PERIOD_S}s -- Ctrl-C to stop")

    request_id = 0
    try:
        while True:
            request_id = (request_id + 1) & 0xFF
            frame = encode_pi_report(
                request_id, PI_ACTION_SCAN_TELETUBBIES, PI_RESULT_NOT_FOUND,
                target_id=0, horizontal_error=0.0, confidence_percent=0)

            t_write = time.monotonic()
            ser.write(frame)
            ser.flush()
            t_flush_returned = time.monotonic()

            queued_at_flush_return = outq_bytes(fd)

            t_drained = None
            deadline = t_flush_returned + POLL_TIMEOUT_S
            while time.monotonic() < deadline:
                if outq_bytes(fd) == 0:
                    t_drained = time.monotonic()
                    break
                time.sleep(0.001)

            flush_ms = (t_flush_returned - t_write) * 1000
            if t_drained is None:
                print(f"[uart_tx_drain_test] id={request_id} flush()={flush_ms:.1f}ms "
                      f"queued_at_return={queued_at_flush_return}B "
                      f"NEVER DRAINED within {POLL_TIMEOUT_S}s!")
            else:
                extra_ms = (t_drained - t_flush_returned) * 1000
                flag = "  <-- flush() lied" if extra_ms > 2.0 else ""
                print(f"[uart_tx_drain_test] id={request_id} flush()={flush_ms:.1f}ms "
                      f"queued_at_return={queued_at_flush_return}B "
                      f"extra_drain_time={extra_ms:.1f}ms{flag}")

            time.sleep(PUSH_PERIOD_S)
    except KeyboardInterrupt:
        print("\n[uart_tx_drain_test] stopped")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
