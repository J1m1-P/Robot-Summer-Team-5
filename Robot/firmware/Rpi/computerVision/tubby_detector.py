"""
Teletubby Detector — scan request handler
================================================================================
Answers one scan request at a time from the arm ESP's dedicated pi_uart link.
The drivetrain owns the search sequence: at each checkpoint it sends a scan
request, and if the reported error is too large it rotates and asks again (up
to ALIGN_MAX_ATTEMPTS times — see robot_sequence_controller.c). This script
just answers each request; it never drives the robot directly.

FLOW
--------------------------------------------------------------------------------
  IDLE      Camera streams to the browser viewer; no YOLO inference runs.
  SCANNING  A request arrived. Sample SCAN_BURST_FRAMES frames, run YOLO on
            each (ignoring identities already in `visited`), and vote per
            identity across the whole burst (needs SCAN_MIN_VOTES agreeing
            frames to count as solid — this is deliberately NOT per-frame-
            closest, so two near-equidistant targets can't split each
            other's votes). The robot is stationary during a scan, so if the
            winning target is centered (|mean error| < ALIGN_THRESHOLD) that
            means the robot is pointed at it right now — flash it and add it
            to `visited`. Then:
              - if that's TARGETS_TO_FIND targets flashed, report
                PI_RESULT_ALL_FOUND and shut this process down — nothing left
                to search for;
              - else if a second, not-yet-flashed teletubby was ALSO solidly
                seen on this chase's first scan (before any turning started),
                report PI_RESULT_REPOSITION instead of reporting done, so the
                ESP turns back to that heading and the next scan finds it;
              - otherwise report done (error 0).
            Send back one report, then return to IDLE.

WIRE CONTRACT (robot_common/pi_action_packet.h)
--------------------------------------------------------------------------------
  ESP -> Pi (PACKET_TYPE_PI_REQUEST): request_id, action, parameter (unused).
  Pi -> ESP (PACKET_TYPE_PI_REPORT): request_id echoed back, result, target_id,
     horizontal_error (-1 left .. +1 right), confidence_percent.
     result is PI_RESULT_OK/NOT_FOUND/CAMERA_FAULT for a real detection.
     PI_RESULT_REPOSITION and PI_RESULT_ALL_FOUND are pure flags -- the ESP
     tracks and undoes its own net commanded rotation (see
     chase_net_rotation_degrees in robot_sequence_controller.c) rather than
     trusting a Pi-reported magnitude, so horizontal_error is ignored for
     both. ALL_FOUND additionally means the drivetrain should skip later
     vision scans while completing the planned route.
"""

import os
import time
import threading
from collections import Counter, defaultdict
from pathlib import Path

import cv2
from flask import Flask, Response, render_template_string
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
CAMERA_INDEX = 1          # ADJUST: 0 if one camera, 1 for built-in + USB
# Capped well above IMGSZ (below) so YOLO's own downsample to IMGSZ isn't
# losing anything -- this just stops the Pi from encoding/streaming a much
# bigger frame than the model ever looks at.
CAMERA_WIDTH  = 640       # ADJUST: keep >= IMGSZ
CAMERA_HEIGHT = 480       # ADJUST: keep >= IMGSZ

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
SCAN_BURST_FRAMES = 5     # ADJUST: frames to sample per PI_REQUEST (bounded, well
                          #         under the ESP's 15s response timeout)
SCAN_MIN_VOTES     = 3    # ADJUST: how many of the burst frames must agree on the
                          #         same identity before we report PI_RESULT_OK
ALIGN_THRESHOLD    = 0.08 # ADJUST: |mean error| below this counts as "centered"
                          #         and triggers a flash (see module docstring)
FLASH_COUNT        = 3    # ADJUST: how many flashes once centered
CHASE_STALE_SECONDS = 10.0 # ADJUST: if this long passes between scans for the
                          #         same identity, treat it as a new chase
                          #         instead of continuing the old one (guards
                          #         against a stale rotation sum bleeding into
                          #         a later, unrelated checkpoint)
TARGETS_TO_FIND    = 2    # only two teletubbies exist — leave at 2. Once this
                          #         many are in `visited`, report
                          #         PI_RESULT_ALL_FOUND and shut down.

# ── training image collection ─────────────────────────────────────────────────
# Saves raw camera frames to disk (no YOLO inference) while IDLE, for building
# up a training set. Off by default -- flip on for a data-collection run,
# leave off for a real match.
COLLECT_IMAGES     = False   # ADJUST: True to save frames while idling
COLLECT_INTERVAL_S = 1.0     # ADJUST: seconds between saved frames
COLLECT_DIR = Path(__file__).resolve().parent / "collected_images"

# ── flash hardware ──────────────────────────────────────────────────────────────
FLASH_PIN     = 18        # ADJUST: BCM GPIO number driving the flash
FLASH_ON_TIME = 0.05      # ADJUST: seconds the flash stays on per pulse

if GPIO is not None:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(FLASH_PIN, GPIO.OUT, initial=GPIO.LOW)

# ── serial link to the ESP32 ──────────────────────────────────────────────────
SERIAL_PORT = "/dev/serial0"        # ADJUST: None = DEV MODE (no link; nothing to answer).
                          #         "COM5" / "/dev/serial0" / "/dev/ttyUSB0" to
                          #         talk to the arm ESP's dedicated pi_uart.
SERIAL_BAUD = 115200      # ADJUST: must match PI_UART_LINK_CONFIG on the ESP
link = RobotLink(SERIAL_PORT, SERIAL_BAUD) if SERIAL_PORT else None

# ── browser view ──────────────────────────────────────────────────────────────
ENABLE_STREAM = True      # ADJUST: True to watch (dev). False on the robot.

# ══════════════════════════════════════════════════════════════════════════════
# YOLO DETECTION
# ══════════════════════════════════════════════════════════════════════════════

model = YOLO(MODEL_PATH)
YOLO_NAME_MAP = {"DP": "dipsy", "LL": "laa_laa", "PO": "po", "TW": "tinky_winky"}

# PiReportPacket.target_id is a single wire byte — map each identity to a stable
# small int. Nothing downstream currently interprets the value beyond logging it.
IDENTITY_TO_TARGET_ID = {"dipsy": 0, "laa_laa": 1, "po": 2, "tinky_winky": 3}

# Identities already flashed. Persists across scan requests (i.e. across
# checkpoints) so a teletubby visible from more than one checkpoint only gets
# flashed once, and so a scan that sees both teletubbies picks the one that
# still needs flashing instead of always the closest-to-center one.
visited = set()

# Tracks the current "chase": whether a second, not-yet-flashed teletubby was
# seen in the same view as `_chase_identity` before any turning started. If
# so, once it's flashed we report PI_RESULT_REPOSITION (a pure flag -- the ESP
# tracks and undoes its own net rotation; the Pi doesn't guess a magnitude).
# See handle_scan_request().
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
    """
    Hard-exit the process (needed to also end the Flask server running in
    main() from this background thread -- stop_event alone wouldn't do it).
    os._exit() skips the normal `finally: GPIO.cleanup()` in __main__, so
    clean up explicitly here first.
    """
    if GPIO is not None:
        GPIO.output(FLASH_PIN, GPIO.LOW)
        GPIO.cleanup()
    os._exit(code)


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
    Sample a short burst of frames, vote on the best not-yet-flashed target,
    flash it if it's already centered, and send back exactly one report.
    `parameter` is decoded but unused (reserved by the wire format for future
    per-scan tuning).
    """
    global _chase_identity, _chase_saw_second, _chase_last_call_time

    # Vote per identity across the whole burst, not per-frame-closest: if two
    # not-yet-flashed teletubbies are near-equidistant, "closest this frame"
    # can flip between them, splitting what would otherwise be solid votes
    # for both and causing a false NOT_FOUND in exactly the case (both
    # visible) the two-target handling below exists for.
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
        publish_frame(frame, pick_target(dets), "SCANNING")

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

    # Continue the running chase for `winner` if we were already centering on
    # it recently; otherwise this is a fresh chase (new identity, or the old
    # one went stale — see CHASE_STALE_SECONDS). `saw_second` only latches on
    # a chase's FIRST scan: net rotation is still zero then, so "undo the
    # whole chase" is guaranteed to return to a heading where it was actually
    # seen. Latching it on a later scan (after we'd already turned partway
    # toward `winner`) would undo too far, past where the second one showed up.
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
            # Nothing left to search for. horizontal_error is ignored by the
            # ESP for this result (see robot_sequence_controller.c) -- it
            # tracks and undoes its own net rotation instead of trusting a
            # Pi-reported magnitude, so the robot returns to roughly its
            # starting heading before moving on to Tower pickup.
            report_error = 0
            report_result = PI_RESULT_ALL_FOUND
            print(f"[SCAN #{request_id}] all {TARGETS_TO_FIND} targets flashed")
        elif _chase_saw_second:
            # A second, not-yet-flashed teletubby was in view before any of
            # the rotating we just did to center on `winner`. Ask the ESP to
            # undo it (horizontal_error ignored here too -- same reason as
            # above) instead of reporting done, so the next scan finds the
            # second one from roughly where it was first seen.
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
# FRAME HAND-OFF — the one thing the control thread and Flask share
# ══════════════════════════════════════════════════════════════════════════════

class FrameBuffer:
    def __init__(self):
        self._cond = threading.Condition()
        self._jpeg = None
        self._seq  = 0

    def publish(self, jpeg_bytes):
        with self._cond:
            self._jpeg = jpeg_bytes
            self._seq += 1
            self._cond.notify_all()

    def get_newer_than(self, last_seq, timeout=1.0):
        with self._cond:
            got_new = self._cond.wait_for(lambda: self._seq != last_seq, timeout)
            if not got_new:
                return None, last_seq
            return self._jpeg, self._seq


frames = FrameBuffer()
stop_event = threading.Event()


# ══════════════════════════════════════════════════════════════════════════════
# CONTROL LOOP
# ══════════════════════════════════════════════════════════════════════════════

def _configure_camera(capture):
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)


cap = cv2.VideoCapture(CAMERA_INDEX)
_configure_camera(cap)


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
    """Owns the camera. Idles (streaming only) until a PI_REQUEST arrives."""
    consecutive_failures = 0
    MAX_FAILURES = 30        # ADJUST: bad reads in a row before trying to reopen

    while not stop_event.is_set():
        if link is not None:
            for msg_type, payload in link.poll():
                if msg_type != PACKET_TYPE_PI_REQUEST:
                    continue
                try:
                    request_id, action, parameter = decode_pi_request(payload)
                except ValueError:
                    continue
                if action == PI_ACTION_SCAN_TELETUBBIES:
                    handle_scan_request(request_id, parameter)

        ok, frame = cap.read()
        if not ok:
            consecutive_failures += 1
            if consecutive_failures >= MAX_FAILURES:
                if _reopen_camera():
                    consecutive_failures = 0
                else:
                    # Reopening didn't help either -- sitting here silently
                    # would just leave the ESP re-requesting scans that time
                    # out for the rest of the match with no visibility into
                    # why. Fail loudly and immediately instead of leaving a
                    # zombie process behind (this is a background thread, so
                    # stop_event alone wouldn't end the Flask process).
                    print("[control_loop] camera unavailable after reopen attempt — giving up.")
                    _shutdown(1)
            time.sleep(0.03)
            continue
        consecutive_failures = 0

        _maybe_collect_image(frame)
        publish_frame(frame, None, "IDLE")
        time.sleep(0.03)

    cap.release()


# ══════════════════════════════════════════════════════════════════════════════
# DISPLAY + WEB SERVER — watch the robot from a browser
# ══════════════════════════════════════════════════════════════════════════════

app = Flask(__name__)


def draw_overlay(frame, detection, label):
    cv2.putText(frame, f"state: {label}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    if detection is None:
        cv2.putText(frame, "no detection", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return frame
    identity, (x, y, w, h), error = detection
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.putText(frame, identity, (x, y - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.circle(frame, (x + w // 2, y + h // 2), 4, (0, 0, 255), -1)
    cx = frame.shape[1] // 2
    cv2.line(frame, (cx, 0), (cx, frame.shape[0]), (255, 255, 0), 1)
    cv2.putText(frame, f"err: {error:+.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return frame


def publish_frame(frame, detection, label):
    if not ENABLE_STREAM:
        return
    display = draw_overlay(frame.copy(), detection, label)
    ok, buffer = cv2.imencode('.jpg', display)
    if ok:
        frames.publish(buffer.tobytes())


@app.route('/')
def index():
    return render_template_string('''
    <html>
    <head>
        <title>Teletubby Detector</title>
        <style>
            body { font-family: monospace; background: #1a1a1a; color: #eee; padding: 20px; }
            img  { display: block; width: 640px; border: 1px solid #444; }
        </style>
    </head>
    <body>
        <h1>Teletubby Detector — request/report (ESP owns the sequence)</h1>
        <img src="/stream">
    </body>
    </html>
    ''')


@app.route('/stream')
def stream():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


def generate_frames():
    last_seq = 0
    while True:
        jpeg, last_seq = frames.get_newer_than(last_seq, timeout=1.0)
        if jpeg is None:
            continue
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')


if __name__ == '__main__':
    worker = threading.Thread(target=control_loop, daemon=True)
    worker.start()

    try:
        if ENABLE_STREAM:
            app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
        else:
            try:
                stop_event.wait()
            except KeyboardInterrupt:
                stop_event.set()
            worker.join(timeout=2.0)
    finally:
        if GPIO is not None:
            GPIO.cleanup()
