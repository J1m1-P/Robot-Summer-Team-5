"""
Teletubby Detector — answers scan requests on the arm ESP's pi_uart link.
The drivetrain owns the search sequence (robot_sequence_controller.c): each
checkpoint requests a scan, re-requesting if the error's too large. This
script only answers — it never drives the robot.

FLOW
  IDLE      Camera runs, no YOLO; state/detections print to the console.
  SCANNING  Inspect one camera frame and select the closest-to-center identity.
            A centered winner (|error| < ALIGN_THRESHOLD) gets
            flashed and added to `visited`, then reports ALL_FOUND
            (TARGETS_TO_FIND flashed), REPOSITION (a second unflashed target
            was also seen — ESP turns back for it), or done. One report per
            request, then back to IDLE.

WIRE (robot_common/pi_action_packet.h)
  ESP->Pi PI_REQUEST: request_id, action, parameter (unused).
  Pi->ESP PI_REPORT:  request_id, result, target_id, horizontal_error
                      (-1 left..+1 right), confidence_percent.
  REPOSITION/ALL_FOUND are flags, not detections: horizontal_error is
  ignored — the ESP tracks/undoes its own rotation instead.
"""

import os
import sys
import time
from collections import Counter, defaultdict
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
    PI_RESULT_REPOSITION,
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
IMGSZ       = 320         # ADJUST: 320 / 480 / 640 — smaller = faster, less accurate
DETECT_CONF = 0.5         # ADJUST: min YOLO confidence

# ── answering one scan request ────────────────────────────────────────────────
SCAN_BURST_FRAMES   = 1    # one picture per scan request
SCAN_MIN_VOTES      = 1    # the single picture must contain a detection
ALIGN_THRESHOLD     = 0.08 # ADJUST: |mean error| below this = "centered" -> flash
FLASH_COUNT         = 2    # ADJUST: flashes fired once centered
CHASE_STALE_SECONDS = 10.0 # ADJUST: gap before a same-identity chase is treated as new
                           #         (stops a stale rotation sum leaking into a later checkpoint)
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
YOLO_NAME_MAP = {"DP": "dipsy", "LL": "laa_laa", "PO": "po", "TW": "tinky_winky"}

# PiReportPacket.target_id is a single wire byte — map each identity to a stable
# small int. Nothing downstream currently interprets the value beyond logging it.
IDENTITY_TO_TARGET_ID = {"dipsy": 0, "laa_laa": 1, "po": 2, "tinky_winky": 3}

# Identities already flashed. Persists across checkpoints so a teletubby seen
# from more than one checkpoint is only flashed once, and a scan seeing both
# picks whichever still needs flashing instead of just the closest one.
visited = set()

# Tracks the current "chase": did a second, unflashed teletubby share the
# view with `_chase_identity` before any turning started? If so, flashing
# the first triggers PI_RESULT_REPOSITION. See handle_scan_request().
_chase_identity = None
_chase_saw_second = False
_chase_last_call_time = 0.0


def yolo_detect_all(frame, frame_w, exclude=None):
    """
    Run YOLO on one frame and return ALL selectable detections, highest-
    confidence first. Each is (identity, (x, y, w, h), error), error in
    -1 (left) .. +1 (right). `exclude` drops identities already in `visited`.
    """
    exclude = exclude or set()
    results = model(frame, conf=DETECT_CONF, imgsz=IMGSZ, verbose=False)
    boxes = results[0].boxes
    if len(boxes) == 0:
        return []

    xyxy = boxes.xyxy.cpu().numpy()
    cls  = boxes.cls.cpu().numpy().astype(int)
    conf = boxes.conf.cpu().numpy()

    dets = []
    for i in conf.argsort()[::-1]:           # highest confidence first
        identity = YOLO_NAME_MAP[model.names[cls[i]]]
        if identity in exclude:
            continue
        x1, y1, x2, y2 = xyxy[i]
        x, y, w, h = int(x1), int(y1), int(x2 - x1), int(y2 - y1)
        error = ((x + w / 2) - frame_w / 2) / (frame_w / 2)
        dets.append((identity, (x, y, w, h), error))
    return dets


def pick_target(detections):
    """Closest-to-center detection, or None if the frame has nothing."""
    if not detections:
        return None
    return min(detections, key=lambda d: abs(d[2]))


# ══════════════════════════════════════════════════════════════════════════════
# ANSWERING ONE SCAN REQUEST
# ══════════════════════════════════════════════════════════════════════════════

def _reset_chase():
    """Clear the running chase state — the current identity is done (flashed,
    lost, or abandoned) so nothing should carry over to whatever's next."""
    global _chase_identity, _chase_saw_second
    _chase_identity = None
    _chase_saw_second = False


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
    lets handle_scan_request's detection/voting/chase logic be exercised
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


def handle_scan_request(request_id, parameter):
    """
    Capture one picture and select the best not-yet-flashed target,
    flash it if it's already centered, and send back exactly one report.
    `parameter` is decoded but unused (reserved by the wire format for future
    per-scan tuning).
    """
    global _chase_identity, _chase_saw_second, _chase_last_call_time

    # Track each identity independently, then choose the closest-to-center one.
    seen_counts = Counter()
    err_by_id = defaultdict(list)
    frames_read = 0
    for _ in range(SCAN_BURST_FRAMES):
        ok, frame = cap.read()
        if not ok:
            continue
        frames_read += 1
        _maybe_collect_image(frame)
        dets = yolo_detect_all(frame, frame.shape[1], exclude=visited)
        for identity in {d[0] for d in dets}:      # count each identity once per frame
            seen_counts[identity] += 1
        for identity, _box, error in dets:
            err_by_id[identity].append(error)
        _report_state("SCANNING", pick_target(dets))

    if frames_read == 0:
        print(f"[SCAN #{request_id}] camera unavailable")
        send_report(request_id, PI_RESULT_CAMERA_FAULT)
        _reset_chase()   # this checkpoint is over either way -- see robot_sequence_controller.c
        return

    solid = [identity for identity, n in seen_counts.items() if n >= SCAN_MIN_VOTES]
    if not solid:
        print(f"[SCAN #{request_id}] no unflashed detection (visited={visited})")
        send_report(request_id, PI_RESULT_NOT_FOUND)
        _reset_chase()
        return

    winner = min(solid, key=lambda i: abs(sum(err_by_id[i]) / len(err_by_id[i])))
    matching_errors = err_by_id[winner]
    mean_error = sum(matching_errors) / len(matching_errors)
    confidence_percent = round(100 * seen_counts[winner] / SCAN_BURST_FRAMES)

    print(f"[SCAN #{request_id}] {winner} err={mean_error:+.3f} conf={confidence_percent}%")

    # Continue the chase for `winner` if we were already centering on it
    # recently; otherwise start fresh (new identity, or stale — see
    # CHASE_STALE_SECONDS). `saw_second` only latches on a chase's first scan,
    # while net rotation is still zero — undoing the whole chase then is
    # guaranteed to land back where the second target was actually seen.
    now = time.time()
    if winner != _chase_identity or (now - _chase_last_call_time) > CHASE_STALE_SECONDS:
        _chase_identity = winner
        _chase_saw_second = len(solid) >= 2
    _chase_last_call_time = now

    report_error, report_result = mean_error, PI_RESULT_OK

    # The robot is stationary during a scan, so a centered reading means it's
    # pointed at the target right now — flash before replying, not after.
    if abs(mean_error) < ALIGN_THRESHOLD:
        for _ in range(FLASH_COUNT):
            flash_once()
        visited.add(winner)
        print(f"[SCAN #{request_id}] flashed {winner}")

        if len(visited) >= TARGETS_TO_FIND:
            # Nothing left to search for — the ESP undoes its own net
            # rotation to return to roughly its starting heading (see
            # WIRE CONTRACT above) before moving on to Tower pickup.
            report_error = 0
            report_result = PI_RESULT_ALL_FOUND
            print(f"[SCAN #{request_id}] all {TARGETS_TO_FIND} targets flashed")
        elif _chase_saw_second:
            # A second unflashed teletubby was in view before we turned to
            # center on `winner` — ask the ESP to undo that turn instead of
            # reporting done, so the next scan finds it from where it was seen.
            report_error = 0
            report_result = PI_RESULT_REPOSITION
            print(f"[SCAN #{request_id}] requesting reposition to re-expose the other one")
        else:
            report_error = 0
        _reset_chase()

    send_report(
        request_id, report_result,
        target_id=IDENTITY_TO_TARGET_ID[winner],
        horizontal_error=report_error,
        confidence_percent=confidence_percent)

    if report_result == PI_RESULT_ALL_FOUND:
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
        identity, _box, error = detection
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
                        handle_scan_request(request_id, parameter)

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

            _maybe_collect_image(frame)
            _report_state("IDLE")
            time.sleep(0.03)
    finally:
        cap.release()


if __name__ == '__main__':
    try:
        control_loop()
    except KeyboardInterrupt:
        pass
    finally:
        if GPIO is not None:
            GPIO.cleanup()
