"""
HSV + shape Teletubby detector

Pipeline per frame (unchanged, still yours to tune):
  BGR -> HSV -> per-colour mask (with saturation gate) -> contours
       -> area filter -> ellipse-fit "oval" confirm -> detection

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

# Full (lower, upper) HSV bound triples per colour, ported from the tuned
# COLOR_RANGES in Rpi/Extra/teletubby_detector_with_HSV.py (tuned at the
# venue under real lighting -- a much better starting point than a guess).
# Padded outward a bit via _widen() below: match-day lighting won't exactly
# match tuning-day lighting, so a slightly wider band trades a little
# false-positive risk for not missing a real tubby.
#
# CAVEATS carried over from the tuned source:
#  - dipsy's and laa_laa's hue ranges overlap on purpose -- upstream relies
#    on a YOLO confirm step to tell them apart by shape, which this
#    HSV-only detector doesn't have. Expect occasional cross-labeling
#    between those two until is_oval()/fill-ratio tuning narrows it down.
#  - po/red was never tuned upstream either (still disabled there) -- kept
#    as the original rough guess, just widened like the rest.
#
# Colour -> teletubby identity: Dipsy is green, Laa-Laa is yellow, Tinky
# Winky is purple, Po is red -- so the colour IS the identity here.
HUE_PAD, SAT_VAL_PAD = 8, 15   # ADJUST: how much to pad the tuned ranges outward


def _widen(lower, upper):
    """Pad a tuned (lower, upper) HSV triple outward by HUE_PAD/SAT_VAL_PAD,
    clipped to OpenCV's valid ranges (H: 0-179, S/V: 0-255)."""
    h_lo, s_lo, v_lo = lower
    h_hi, s_hi, v_hi = upper
    return (
        (max(0, h_lo - HUE_PAD), max(0, s_lo - SAT_VAL_PAD), max(0, v_lo - SAT_VAL_PAD)),
        (min(179, h_hi + HUE_PAD), min(255, s_hi + SAT_VAL_PAD), min(255, v_hi + SAT_VAL_PAD)),
    )


COLOR_HUES = {
    "purple": [_widen((127, 24, 23), (154, 174, 199))],    # tinky_winky
    "green":  [_widen((23, 26, 15), (73, 195, 207))],       # dipsy
    "yellow": [_widen((15, 26, 15), (57, 195, 207))],       # laa_laa
    "red": [                                                # po -- untuned guess, widened
        _widen((0, 100, 60), (10, 255, 255)),
        _widen((170, 100, 60), (179, 255, 255)),            # red wraps 0/179, needs two ranges
    ],
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
SCAN_BURST_FRAMES   = 1    # one picture per scan request
SCAN_MIN_VOTES      = 1    # the single picture must contain a detection
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

# Small kernel for color_mask()'s noise cleanup -- big enough to erase
# single-pixel speckle without eating into a real tubby-sized blob.
_MORPH_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))


def color_mask(hsv, hue_ranges):
    """Return a binary mask for one colour: white where the pixel falls
    inside any of the given (lower, upper) HSV bound triples."""
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for (lower, upper) in hue_ranges:
        mask |= cv2.inRange(hsv, np.array(lower), np.array(upper))

    # OPEN (erode then dilate) strips lone speckle pixels; CLOSE (dilate
    # then erode) fills small holes -- e.g. a specular highlight on the
    # felt/plastic -- so a real tubby's blob doesn't fragment into several
    # under-MIN_AREA pieces.
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, _MORPH_KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, _MORPH_KERNEL)
    return mask


def is_oval(contour):
    """Return (ok, info) — does this contour look like an oval tubby body?"""
    if len(contour) < 5:          # fitEllipse needs >= 5 points
        return False, None

    ellipse = cv2.fitEllipse(contour)
    (cx, cy), (axis_a, axis_b), angle = ellipse

    major, minor = max(axis_a, axis_b), min(axis_a, axis_b)
    if minor <= 0:                 # degenerate (near-flat) ellipse -- guard the divide below
        return False, ellipse

    aspect = major / minor                             # 1.0 = circle, bigger = more elongated
    ellipse_area = np.pi * (major / 2) * (minor / 2)
    fill_ratio = cv2.contourArea(contour) / ellipse_area  # how well the blob fills its ellipse

    ok = fill_ratio >= MIN_FILL_RATIO and aspect <= MAX_ASPECT
    return ok, ellipse


OVERLAP_IOU = 0.3   # ADJUST: bounding-box IoU above this = "same physical blob"


def _bbox_iou(box_a, box_b):
    """Intersection-over-union of two (x1, y1, x2, y2) boxes -- used to tell
    whether two colours' blobs are claiming the same physical spot."""
    inter_x1, inter_y1 = max(box_a[0], box_b[0]), max(box_a[1], box_b[1])
    inter_x2, inter_y2 = min(box_a[2], box_b[2]), min(box_a[3], box_b[3])
    intersection = max(0, inter_x2 - inter_x1) * max(0, inter_y2 - inter_y1)
    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    union = area_a + area_b - intersection
    return intersection / union if union > 0 else 0.0


def detect(frame):
    """Return a list of detections: (color, (cx, cy), ellipse). If two
    colours both claim overlapping blobs, only the larger + more oval one
    is kept -- that's almost always the same tubby catching a second
    colour's hue at its edges, not two separate tubbies in the same spot.
    """
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Collect every colour's candidates first, before deciding which
    # overlapping ones to drop -- can't resolve a conflict until both
    # sides of it are known.
    candidates = []   # each: (color, ellipse, bbox, score)
    for color, hue_ranges in COLOR_HUES.items():
        mask = color_mask(hsv, hue_ranges)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area < MIN_AREA:
                continue
            ok, ellipse = is_oval(c)
            if not ok:
                continue
            x, y, w, h = cv2.boundingRect(c)
            major, minor = max(ellipse[1]), min(ellipse[1])
            # Bigger blob AND closer to circular (minor/major -> 1.0) both
            # push the score up, matching "larger / more oval" combined.
            score = area * (minor / major if major > 0 else 0.0)
            candidates.append((color, ellipse, (x, y, x + w, y + h), score))

    # Drop the lower-scoring half of any cross-colour pair whose bounding
    # boxes overlap. Two blobs of the SAME colour are left alone -- that's
    # two real, separate blobs, not a conflicting claim.
    keep = [True] * len(candidates)
    for i in range(len(candidates)):
        if not keep[i]:
            continue
        for j in range(i + 1, len(candidates)):
            if not keep[j] or candidates[i][0] == candidates[j][0]:
                continue
            if _bbox_iou(candidates[i][2], candidates[j][2]) < OVERLAP_IOU:
                continue
            if candidates[i][3] >= candidates[j][3]:
                keep[j] = False
            else:
                keep[i] = False
                break   # i is out -- no need to check it against the rest

    detections = []
    for (color, ellipse, _bbox, _score), kept in zip(candidates, keep):
        if not kept:
            continue
        (cx, cy), _, _ = ellipse
        detections.append((color, (int(cx), int(cy)), ellipse))
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
    Capture one picture and select the best not-yet-flashed target,
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
