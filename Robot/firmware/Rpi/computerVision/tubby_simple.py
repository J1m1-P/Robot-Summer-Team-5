"""
Teletubby Detector — SIMPLE FLASH MODE.
FLOW
  IDLE      Camera runs, no YOLO; state/detections print to the console.
  SCANNING  Inspect one camera frame for a teletubby. Identity (which
            character it is) is ignored entirely -- every detection is
            treated as the same one "teletubby" class.
            A detection is flashed immediately and counted, then reports
            ALL_FOUND (TARGETS_TO_FIND flashed) or plain OK. One report per
            request, then back to IDLE.

WIRE (robot_common/pi_action_packet.h) -- identical to tubby_detector.py.
  ESP->Pi PI_REQUEST: request_id, action, parameter (unused).
  Pi->ESP PI_REPORT:  request_id, result, target_id, horizontal_error
                      (always 0 here -- nothing measures how centered the
                      winner was), confidence_percent.
  ALL_FOUND is a flag, not a detection: horizontal_error is ignored
"""

import os
import sys
import time
from pathlib import Path

import cv2
from ultralytics import YOLO
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


# ══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION — everything you tune lives here
# ══════════════════════════════════════════════════════════════════════════════
# ── camera ────────────────────────────────────────────────────────────────────
CAMERA_INDEX = 0          # ADJUST: 0 if one camera, 1 for built-in + USB
CAMERA_WIDTH  = 640       # ADJUST: keep >= IMGSZ, no point capturing bigger
CAMERA_HEIGHT = 480       # ADJUST: than what YOLO's own downsample will use

# ── model / detection ─────────────────────────────────────────────────────────
# Override for development; production defaults to the model committed beside
# this directory at Rpi/best_ncnn_model.
MODEL_PATH  = os.environ.get(
    "TUBBY_MODEL_PATH",
    str(Path(__file__).resolve().parent.parent / "best_ncnn_model"),
)
IMGSZ       = 640         # ADJUST: 320 / 480 / 640 — smaller = faster, less accurate
DETECT_CONF = 0.5         # ADJUST: min YOLO confidence

# ── answering one scan request ────────────────────────────────────────────────
SCAN_BURST_FRAMES   = 1    # one picture per scan request
SCAN_MIN_VOTES      = 1    # the single picture must contain a detection
FLASH_COUNT         = 2    # ADJUST: flashes fired once a winner is voted
TARGETS_TO_FIND     = 2    # only two teletubbies exist — leave at 2

# ── training image collection ─────────────────────────────────────────────────
# Saves raw camera frames to disk (no YOLO inference) while IDLE, for building
# up a training set. Off by default -- flip on for a data-collection run,
# leave off for a real match.
COLLECT_IMAGES     = False   # ADJUST: True to save frames while idling
COLLECT_INTERVAL_S = 1.0     # ADJUST: seconds between saved frames
COLLECT_DIR = Path(__file__).resolve().parent / "collected_images"

# ── flash hardware ──────────────────────────────────────────────────────────────
FLASH_PIN     = 18        # ADJUST: BCM GPIO number driving the flash
FLASH_ON_TIME = 0.5      # ADJUST: seconds the flash stays on per pulse

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

# ══════════════════════════════════════════════════════════════════════════════
# YOLO DETECTION
# ══════════════════════════════════════════════════════════════════════════════

print(f"[startup] loading model from {MODEL_PATH} ...")
try:
    model = YOLO(MODEL_PATH)
except Exception as e:
    print(f"[startup] FAILED to load model from {MODEL_PATH}: {e}")
    raise
print(f"[startup] model loaded, classes={list(model.names.values())}")

# Number of teletubbies flashed so far. Identity (which character) is never
# considered -- every detection is the same one class, so this is a plain
# count, not a set of identities seen.
found_count = 0


def yolo_detect_all(frame, frame_w):
    """
    Run YOLO on one frame and return ALL detections, highest-confidence
    first, as ((x, y, w, h), horizontal_error) pairs, error in
    -1 (left) .. +1 (right). Identity is ignored -- any detected box counts.
    """
    # Run the model once on this frame; bail early if it found nothing at all.
    results = model(frame, conf=DETECT_CONF, imgsz=IMGSZ, verbose=False)
    boxes = results[0].boxes
    if len(boxes) == 0:
        return []

    # Pull the raw box data out of the torch tensors as plain numpy arrays.
    xyxy = boxes.xyxy.cpu().numpy()
    conf = boxes.conf.cpu().numpy()

    # Build one (box, horizontal_error) tuple per detection, highest
    # confidence first.
    dets = []
    for i in conf.argsort()[::-1]:           # highest confidence first
        x1, y1, x2, y2 = xyxy[i]
        x, y, w, h = int(x1), int(y1), int(x2 - x1), int(y2 - y1)
        error = ((x + w / 2) - frame_w / 2) / (frame_w / 2)   # -1 left .. +1 right
        dets.append(((x, y, w, h), error))
    return dets


def pick_target(detections):
    """Closest-to-center detection, or None if the frame has nothing."""
    if not detections:
        return None
    return min(detections, key=lambda d: abs(d[1]))


# ══════════════════════════════════════════════════════════════════════════════
# ANSWERING ONE SCAN REQUEST
# ══════════════════════════════════════════════════════════════════════════════

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
    """Exit once every target is flashed -- nothing left to search for. A
    normal SystemExit is enough now that control_loop runs on the main
    thread; the `finally` blocks in control_loop()/__main__ still fire."""
    if GPIO is not None:
        GPIO.output(FLASH_PIN, GPIO.LOW)
    sys.exit(code)


def send_report(request_id, result, target_id=0, horizontal_error=0.0,
                 confidence_percent=0):
    """
    Send one PiReportPacket, or just print it in dev mode (link is None) --
    lets handle_scan_request_simple's detection/voting logic be exercised
    (e.g. by a standalone test harness) without a real serial link.
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
    Sample a short burst of frames, check whether a teletubby was seen in
    enough of them to count as solid (filtering single-frame noise -- only
    one teletubby is ever in frame at a time, so this is a presence check,
    not a contest between candidates), and flash it immediately -- no
    centering check, no identity, no cross-request chase tracking, since
    every call either flashes on the spot or reports NOT_FOUND/CAMERA_FAULT.
    `parameter` is decoded but unused (reserved by the wire format for future
    per-scan tuning).
    """
    global found_count

    # CHUNK 1 — capture one picture and check it for a detection (identity doesn't
    # matter, just presence).
    frames_seen = 0
    frames_read = 0
    for _ in range(SCAN_BURST_FRAMES):
        ok, frame = cap.read()
        if not ok:
            continue
        frames_read += 1
        _maybe_collect_image(frame)
        dets = yolo_detect_all(frame, frame.shape[1])
        if dets:
            frames_seen += 1
        _report_state("SCANNING", pick_target(dets))

    # CHUNK 2 — the camera did not produce a usable picture.
    if frames_read == 0:
        print(f"[SCAN #{request_id}] camera unavailable")
        send_report(request_id, PI_RESULT_CAMERA_FAULT)
        return

    # CHUNK 3 — the single picture must contain a detection.
    if frames_seen < SCAN_MIN_VOTES:
        print(f"[SCAN #{request_id}] no detection (found so far={found_count})")
        send_report(request_id, PI_RESULT_NOT_FOUND)
        return

    confidence_percent = round(100 * frames_seen / SCAN_BURST_FRAMES)

    # CHUNK 4 — detection locked in: flash it and count it. There's no
    # identity to dedupe on, so this relies on the caller only requesting a
    # scan once per physical teletubby (e.g. after repositioning).
    for _ in range(FLASH_COUNT):
        flash_once()
    found_count += 1
    print(f"[SCAN #{request_id}] flashed teletubby #{found_count} conf={confidence_percent}%")

    # CHUNK 5 — tell the ESP whether that was the last target or not, so it
    # knows whether to send another scan request later or move on.
    if found_count >= TARGETS_TO_FIND:
        # Nothing left to search for — the ESP moves on to Tower pickup.
        result = PI_RESULT_ALL_FOUND
        print(f"[SCAN #{request_id}] all {TARGETS_TO_FIND} targets flashed")
    else:
        result = PI_RESULT_OK

    send_report(
        request_id, result,
        target_id=0,
        horizontal_error=0.0,
        confidence_percent=confidence_percent)

    if result == PI_RESULT_ALL_FOUND:
        # Give the report a moment to actually go out over the wire first.
        time.sleep(0.1)
        print("[control_loop] all targets found — shutting down")
        _shutdown(0)


# ══════════════════════════════════════════════════════════════════════════════
# CONSOLE STATE REPORTING — replaces the old Flask/browser view
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
        _box, error = detection
        print(f"  see teletubby err={error:+.3f}")


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
# Camera's open and the model (loaded above, before this point) is warm, so
# tell the arm ESP it's safe to start relaying scan requests. Repeated at
# READY_REPEAT_INTERVAL_S -- like arm_action_dispatcher.cpp's own startup
# beacon -- so one lost byte on this link can't strand the ESP waiting
# forever; it stops once the first request proves the ESP already knows.
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


_last_collect_time = 0.0
_collect_count = 0


def _maybe_collect_image(frame):
    """
    Saves one raw idle frame to COLLECT_DIR at most every COLLECT_INTERVAL_S
    -- no YOLO involved, just building up a training set. No-op unless
    COLLECT_IMAGES is on.
    """
    global _last_collect_time, _collect_count
    if not COLLECT_IMAGES:
        return
    now = time.time()
    if now - _last_collect_time < COLLECT_INTERVAL_S:
        return
    _last_collect_time = now
    COLLECT_DIR.mkdir(parents=True, exist_ok=True)
    _collect_count += 1
    filename = COLLECT_DIR / f"{int(now)}_{_collect_count:05d}.jpg"
    cv2.imwrite(str(filename), frame)
    print(f"[collect] saved {filename.name} ({_collect_count} total)")


def control_loop():
    """Owns the camera. Idles until a PI_REQUEST arrives."""
    global _ready_pending, _last_ready_sent_time
    consecutive_failures = 0
    MAX_FAILURES = 30        # ADJUST: bad reads in a row before trying to reopen

    try:
        while True:
            # CHUNK 1 — everything UART-related: nudge the readiness beacon
            # if the ESP hasn't acknowledged it yet, then answer any scan
            # requests that arrived since the last time round the loop.
            if link is not None:
                if _ready_pending:
                    now = time.time()
                    if now - _last_ready_sent_time >= READY_REPEAT_INTERVAL_S:
                        _last_ready_sent_time = now
                        link.send_pi_ready()

                for msg_type, payload in link.poll():
                    if msg_type != PACKET_TYPE_PI_REQUEST:
                        continue    # only scan requests arrive on this link
                    try:
                        request_id, action, parameter = decode_pi_request(payload)
                    except ValueError:
                        continue
                    # The ESP only sends a request once it already knows the
                    # Pi is ready, so any request proves the beacon landed.
                    _ready_pending = False
                    if action == PI_ACTION_SCAN_TELETUBBIES:
                        handle_scan_request_simple(request_id, parameter)

            # CHUNK 2 — one idle-loop camera read. A failed read just counts
            # against the retry budget; MAX_FAILURES in a row triggers a
            # reopen attempt, and a failed reopen gives up for good.
            ok, frame = cap.read()
            if not ok:
                consecutive_failures += 1
                if consecutive_failures >= MAX_FAILURES:
                    if _reopen_camera():
                        consecutive_failures = 0
                    else:
                        # Reopening didn't help either -- sitting here
                        # silently would just leave the ESP re-requesting
                        # scans that time out for the rest of the match with
                        # no visibility into why.
                        print("[control_loop] camera unavailable after reopen attempt — giving up.")
                        _shutdown(1)
                time.sleep(0.03)
                continue
            consecutive_failures = 0

            # CHUNK 3 — read succeeded and no scan was requested this tick:
            # genuinely idle. Optionally bank the frame for training data,
            # log the state, and loop again.
            _maybe_collect_image(frame)
            _report_state("IDLE")
            time.sleep(0.03)
    finally:
        cap.release()


if __name__ == '__main__':
    try:
        control_loop()
    except KeyboardInterrupt:
        pass    # Ctrl+C on the bench -- still fall through to GPIO cleanup below
    finally:
        if GPIO is not None:
            GPIO.cleanup()
