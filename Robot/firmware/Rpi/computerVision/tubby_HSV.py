"""
HSV + shape Teletubby detector — answers scan requests on the arm ESP's
pi_uart link, same wire protocol and control-loop shell as
tubby_detector_simple.py (burst-voted, single-frame-decides, no centering
gate -- see that file's docstring for why). The only thing that differs is
the detection backend: color_mask()/is_oval()/detect() below (HSV mask +
ellipse fit) instead of YOLO.

Only run one of tubby_detector.py / tubby_detector_simple.py /
tubby_HSV.py at a time -- each claims the camera and serial port
exclusively.

Pipeline per frame (unchanged, still yours to tune):
  BGR -> HSV -> per-colour mask (with saturation gate) -> contours
       -> area filter -> ellipse-fit "oval" confirm -> detection

Fill in the TODOs in color_mask()/is_oval(). The plumbing below (wire
protocol, burst voting, flash, control loop) is done; the tuning + the two
real CV decisions (hue bounds, what counts as "oval") are still yours.

RUNNING
  python tubby_HSV.py          real run: answers scan requests over SERIAL_PORT
  python tubby_HSV.py --bench  visual bench mode: cv2.imshow preview, no UART,
                                for tuning hue/oval thresholds on your laptop
"""

import sys
import time
from collections import Counter

import cv2
import numpy as np
from uart_link import (
    RobotLink,
    PACKET_TYPE_PI_REQUEST,
    decode_pi_request,
    PI_ACTION_SCAN_TELETUBBIES,
    PI_RESULT_OK,
    PI_RESULT_NOT_FOUND,
    PI_RESULT_CAMERA_FAULT,
    PI_RESULT_ALL_FOUND,
)   # the ESP32 serial link

try:
    import RPi.GPIO as GPIO       # only present on the Pi itself
except (ImportError, RuntimeError):
    GPIO = None                   # dev machine — flash_once() just logs


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

# ── camera ────────────────────────────────────────────────────────────────────
CAMERA_INDEX = 0          # ADJUST: 0 if one camera, 1 for built-in + USB
CAMERA_WIDTH  = 640       # ADJUST
CAMERA_HEIGHT = 480       # ADJUST

# Saturation / value gate: kills the white background AND the grey rock.
# White  -> very low S.   Rock (grey/brown) -> low-ish S.
# TODO: tune S_MIN so the rock never survives but the tubbies always do.
S_MIN = 100
V_MIN = 60

# Hue is 0-179 in OpenCV. Red wraps around 0/179 so it needs TWO ranges.
# TODO: point a tubby at the camera, print the HSV of its blob, and tighten
#       these bounds. Values below are rough starting guesses only.
# Colour -> teletubby identity: Dipsy is green, Laa-Laa is yellow, Tinky
# Winky is purple, Po is red -- so the colour IS the identity here.
COLOR_HUES = {
    "green":  [(35, 85)],
    "yellow": [(20, 35)],
    "purple": [(125, 160)],
    "red":    [(0, 10), (170, 179)],   # two-piece because red wraps
}

MIN_AREA = 800          # ignore blobs smaller than this (pixels). TODO: tune.

# "Oval" acceptance thresholds. TODO: decide + tune these.
#   - fill ratio  = contour area / ellipse area   (how well an ellipse fits)
#   - aspect      = major axis / minor axis        (reject long thin things)
MIN_FILL_RATIO = 0.70
MAX_ASPECT = 3.0

# ── answering one scan request ────────────────────────────────────────────────
# Same numbers as tubby_detector_simple.py on purpose -- keeps the two
# detectors comparable when running them back-to-back.
SCAN_BURST_FRAMES   = 5    # ADJUST: frames sampled per request
SCAN_MIN_VOTES      = 3    # ADJUST: burst frames that must agree before flashing
FLASH_COUNT         = 2    # ADJUST: flashes fired once a winner is voted
TARGETS_TO_FIND     = 2    # only two teletubbies exist — leave at 2

# ── flash hardware ──────────────────────────────────────────────────────────────
FLASH_PIN     = 18        # ADJUST: BCM GPIO number driving the flash
FLASH_ON_TIME = 0.5       # ADJUST: seconds the flash stays on per pulse

if GPIO is not None:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(FLASH_PIN, GPIO.OUT, initial=GPIO.LOW)
    print(f"[startup] flash ready on GPIO{FLASH_PIN}")
else:
    print("[startup] no RPi.GPIO -- dev machine, flashes will just log")

# ── serial link to the ESP32 ──────────────────────────────────────────────────
SERIAL_PORT = "/dev/serial0"  # ADJUST: "COM5"/"/dev/ttyUSB0"/etc.; None = dev mode, no link
SERIAL_BAUD = 115200          # ADJUST: must match PI_UART_LINK_CONFIG on the ESP
if SERIAL_PORT:
    link = RobotLink(SERIAL_PORT, SERIAL_BAUD)
else:
    link = None
    print("[startup] SERIAL_PORT is None -- dev mode, reports print to console only")

print(f"[startup] HSV detector ready, colors={list(COLOR_HUES.keys())}")

# PiReportPacket.target_id is a single wire byte — map each identity to a stable
# small int. Nothing downstream currently interprets the value beyond logging it.
COLOR_TO_IDENTITY = {
    "green": "dipsy",
    "yellow": "laa_laa",
    "red": "po",
    "purple": "tinky_winky",
}
IDENTITY_TO_TARGET_ID = {"dipsy": 0, "laa_laa": 1, "po": 2, "tinky_winky": 3}

# Identities already flashed. Persists across checkpoints so a teletubby seen
# from more than one checkpoint is only flashed once.
visited = set()


# ---------------------------------------------------------------------------
# Core detection steps (yours to tune -- unchanged by the wire-protocol
# integration below)
# ---------------------------------------------------------------------------

def color_mask(hsv, hue_ranges):
    """Return a binary mask for one colour, with the saturation/value gate applied."""
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for (h_lo, h_hi) in hue_ranges:
        lower = np.array([h_lo, S_MIN, V_MIN])
        upper = np.array([h_hi, 255, 255])
        mask |= cv2.inRange(hsv, lower, upper)

    # TODO: clean up salt-and-pepper noise here.
    # Hint: cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) then MORPH_CLOSE.
    return mask


def is_oval(contour):
    """Return (ok, info) — does this contour look like an oval tubby body?"""
    if len(contour) < 5:          # fitEllipse needs >= 5 points
        return False, None

    ellipse = cv2.fitEllipse(contour)
    (cx, cy), (axis_a, axis_b), angle = ellipse

    # TODO: compute the two metrics and compare against the thresholds.
    #   major, minor = max(axis_a, axis_b), min(axis_a, axis_b)
    #   aspect = major / minor
    #   ellipse_area = np.pi * (major/2) * (minor/2)
    #   fill_ratio = cv2.contourArea(contour) / ellipse_area
    # Return True only if fill_ratio >= MIN_FILL_RATIO and aspect <= MAX_ASPECT.

    ok = False   # TODO: replace with the real test
    return ok, ellipse


def detect(frame):
    """Return a list of detections: (color, (cx, cy), ellipse)."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detections = []

    for color, hue_ranges in COLOR_HUES.items():
        mask = color_mask(hsv, hue_ranges)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) < MIN_AREA:
                continue
            ok, ellipse = is_oval(c)
            if not ok:
                continue
            (cx, cy), _, _ = ellipse
            detections.append((color, (int(cx), int(cy)), ellipse))

    # TODO (optional): if two colours claim overlapping blobs, keep the
    #       larger / more oval one so you don't double-report a tubby.
    return detections


# ══════════════════════════════════════════════════════════════════════════════
# WIRE-PROTOCOL INTEGRATION — mirrors tubby_detector_simple.py from here down
# ══════════════════════════════════════════════════════════════════════════════

def hsv_detect_all(frame, frame_w, exclude=None):
    """
    Adapts detect()'s raw (color, (cx, cy), ellipse) output to the identity/
    error shape the scan handler and console logging use, dropping any color
    without a mapped identity and any identity already in `exclude`.
    """
    exclude = exclude or set()
    dets = []
    for color, (cx, cy), _ellipse in detect(frame):
        identity = COLOR_TO_IDENTITY.get(color)
        if identity is None or identity in exclude:
            continue
        error = (cx - frame_w / 2) / (frame_w / 2)
        dets.append((identity, (cx, cy), error))
    return dets


def pick_target(detections):
    """Closest-to-center detection, or None if the frame has nothing."""
    if not detections:
        return None
    return min(detections, key=lambda d: abs(d[2]))


def flash_once():
    """Fire the onboard flash once via GPIO18."""
    if GPIO is None:
        print("[FLASH] (Pi-side, no GPIO — dev machine)")
        return
    GPIO.output(FLASH_PIN, GPIO.HIGH)
    time.sleep(FLASH_ON_TIME)
    GPIO.output(FLASH_PIN, GPIO.LOW)
    print("[FLASH] (Pi-side)")


def _shutdown(code):
    """Exit once every target is flashed -- nothing left to search for."""
    if GPIO is not None:
        GPIO.output(FLASH_PIN, GPIO.LOW)
    sys.exit(code)


def send_report(request_id, result, target_id=0, horizontal_error=0.0,
                 confidence_percent=0):
    """
    Send one PiReportPacket, or just print it in dev mode (link is None) --
    lets handle_scan_request_simple's detection/voting logic be exercised
    without a real serial link.
    """
    if link is None:
        print(f"[TX] report result={result} target={target_id} "
              f"err={horizontal_error:+.3f} conf={confidence_percent}%")
        return
    link.send_pi_report(
        request_id, PI_ACTION_SCAN_TELETUBBIES, result,
        target_id=target_id, horizontal_error=horizontal_error,
        confidence_percent=confidence_percent)


def handle_scan_request_simple(request_id, parameter):
    """
    Sample a short burst of frames, vote on the best not-yet-flashed target,
    and flash it immediately -- no centering check, no cross-request chase
    tracking, since every call either flashes on the spot or reports
    NOT_FOUND/CAMERA_FAULT. `parameter` is decoded but unused (reserved by
    the wire format for future per-scan tuning).
    """
    seen_counts = Counter()
    frames_read = 0
    for _ in range(SCAN_BURST_FRAMES):
        ok, frame = cap.read()
        if not ok:
            continue
        frames_read += 1
        dets = hsv_detect_all(frame, frame.shape[1], exclude=visited)
        for identity in {d[0] for d in dets}:      # count each identity once per frame
            seen_counts[identity] += 1
        _report_state("SCANNING", pick_target(dets))

    if frames_read == 0:
        print(f"[SCAN #{request_id}] camera unavailable")
        send_report(request_id, PI_RESULT_CAMERA_FAULT)
        return

    # Only one teletubby is ever in frame at a time, so at most one identity
    # can reach SCAN_MIN_VOTES.
    solid = [identity for identity, n in seen_counts.items() if n >= SCAN_MIN_VOTES]
    if not solid:
        print(f"[SCAN #{request_id}] no unflashed detection (visited={visited})")
        send_report(request_id, PI_RESULT_NOT_FOUND)
        return

    winner = solid[0]
    confidence_percent = round(100 * seen_counts[winner] / SCAN_BURST_FRAMES)

    for _ in range(FLASH_COUNT):
        flash_once()
    visited.add(winner)
    print(f"[SCAN #{request_id}] flashed {winner} conf={confidence_percent}%")

    if len(visited) >= TARGETS_TO_FIND:
        # Nothing left to search for — the ESP moves on to Tower pickup.
        result = PI_RESULT_ALL_FOUND
        print(f"[SCAN #{request_id}] all {TARGETS_TO_FIND} targets flashed")
    else:
        result = PI_RESULT_OK

    send_report(
        request_id, result,
        target_id=IDENTITY_TO_TARGET_ID[winner],
        horizontal_error=0.0,
        confidence_percent=confidence_percent)

    if result == PI_RESULT_ALL_FOUND:
        # Give the report a moment to actually go out over the wire first.
        time.sleep(0.1)
        print("[control_loop] all targets found — shutting down")
        _shutdown(0)


# ══════════════════════════════════════════════════════════════════════════════
# CONSOLE STATE REPORTING
# ══════════════════════════════════════════════════════════════════════════════

_last_reported_state = None


def _report_state(label, detection=None):
    """
    Print state transitions (IDLE <-> SCANNING) once each, and print a
    detection only when one is actually seen -- no per-frame spam while
    idling or while a burst frame comes up empty.
    """
    global _last_reported_state
    if label != _last_reported_state:
        print(f"[{label}]")
        _last_reported_state = label
    if detection is not None:
        identity, _pos, error = detection
        print(f"  see {identity} err={error:+.3f}")


# ══════════════════════════════════════════════════════════════════════════════
# CONTROL LOOP
# ══════════════════════════════════════════════════════════════════════════════

def _configure_camera(capture):
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)


print(f"[startup] opening camera index {CAMERA_INDEX} ...")
cap = cv2.VideoCapture(CAMERA_INDEX)
_configure_camera(cap)
if cap.isOpened():
    print(f"[startup] camera ready ({CAMERA_WIDTH}x{CAMERA_HEIGHT})")
else:
    print(f"[startup] WARNING: camera index {CAMERA_INDEX} did not open -- "
          f"check it's connected; control_loop will keep retrying")

# ── readiness beacon ────────────────────────────────────────────────────────
# Camera's open, so tell the arm ESP it's safe to start relaying scan
# requests. Repeated at READY_REPEAT_INTERVAL_S so one lost byte on this
# link can't strand the ESP waiting forever; stops once the first request
# proves the ESP already knows.
READY_REPEAT_INTERVAL_S = 0.2
_ready_pending = True
_last_ready_sent_time = 0.0


def _reopen_camera():
    """
    Try to recover from a dead camera by reopening it once. Returns True only
    if the fresh handle can actually read a frame.
    """
    global cap
    print("[control_loop] camera unresponsive — attempting to reopen")
    cap.release()
    cap = cv2.VideoCapture(CAMERA_INDEX)
    _configure_camera(cap)
    for _ in range(5):
        ok, _ = cap.read()
        if ok:
            print("[control_loop] camera reopened successfully")
            return True
        time.sleep(0.1)
    return False


def control_loop():
    """Owns the camera. Idles until a PI_REQUEST arrives."""
    global _ready_pending, _last_ready_sent_time
    consecutive_failures = 0
    MAX_FAILURES = 30        # ADJUST: bad reads in a row before trying to reopen

    try:
        while True:
            if link is not None:
                if _ready_pending:
                    now = time.time()
                    if now - _last_ready_sent_time >= READY_REPEAT_INTERVAL_S:
                        _last_ready_sent_time = now
                        link.send_pi_ready()

                for msg_type, payload in link.poll():
                    if msg_type != PACKET_TYPE_PI_REQUEST:
                        continue
                    try:
                        request_id, action, parameter = decode_pi_request(payload)
                    except ValueError:
                        continue
                    # The ESP only sends a request once it already knows the
                    # Pi is ready, so any request proves the beacon landed.
                    _ready_pending = False
                    if action == PI_ACTION_SCAN_TELETUBBIES:
                        handle_scan_request_simple(request_id, parameter)

            ok, _frame = cap.read()
            if not ok:
                consecutive_failures += 1
                if consecutive_failures >= MAX_FAILURES:
                    if _reopen_camera():
                        consecutive_failures = 0
                    else:
                        print("[control_loop] camera unavailable after reopen attempt — giving up.")
                        _shutdown(1)
                time.sleep(0.03)
                continue
            consecutive_failures = 0

            _report_state("IDLE")
            time.sleep(0.03)
    finally:
        cap.release()


# ---------------------------------------------------------------------------
# Bench mode (for visually tuning color_mask()/is_oval() on your laptop --
# no UART, no flash, just a preview window)
# ---------------------------------------------------------------------------

def bench_preview():
    bench_cap = cv2.VideoCapture(CAMERA_INDEX)
    while True:
        ret, frame = bench_cap.read()
        if not ret:
            break

        for color, (cx, cy), ellipse in detect(frame):
            cv2.ellipse(frame, ellipse, (0, 255, 255), 2)
            cv2.putText(frame, color, (cx - 20, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("hsv+shape", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    bench_cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    if "--bench" in sys.argv:
        bench_preview()
    else:
        try:
            control_loop()
        except KeyboardInterrupt:
            pass
        finally:
            if GPIO is not None:
                GPIO.cleanup()
