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
            each, and vote on the most consistent detection (needs
            SCAN_MIN_VOTES agreeing frames to count). The robot is stationary
            during a scan, so if the winning target is centered
            (|mean error| < ALIGN_THRESHOLD) that means the robot is pointed
            at it right now — flash it before replying. Send back one report,
            then return to IDLE.

WIRE CONTRACT (robot_common/pi_action_packet.h)
--------------------------------------------------------------------------------
  ESP -> Pi (PACKET_TYPE_PI_REQUEST): request_id, action, parameter (unused).
  Pi -> ESP (PACKET_TYPE_PI_REPORT): request_id echoed back, result
     (OK / NOT_FOUND / CAMERA_FAULT), target_id, horizontal_error
     (-1 left .. +1 right), confidence_percent.
"""

import time
import threading
from collections import Counter

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

# ── model / detection ─────────────────────────────────────────────────────────
MODEL_PATH  = r"E:\runs\detect\train-9\weights\best_ncnn_model"  # ADJUST: .pt or ncnn folder
IMGSZ       = 320         # ADJUST: 320 / 480 / 640 — smaller = faster, less accurate
DETECT_CONF = 0.5         # ADJUST: min YOLO confidence

# ── answering one scan request ────────────────────────────────────────────────
SCAN_BURST_FRAMES = 5     # ADJUST: frames to sample per PI_REQUEST (bounded, well
                          #         under the ESP's 15s response timeout)
SCAN_MIN_VOTES     = 3    # ADJUST: how many of the burst frames must agree on the
                          #         same identity before we report PI_RESULT_OK
ALIGN_THRESHOLD    = 0.08 # ADJUST: |mean error| below this counts as "centered"
                          #         and triggers a flash (see module docstring)

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


def yolo_detect_all(frame, frame_w):
    """
    Run YOLO on one frame and return ALL detections, highest-confidence first.
    Each is (identity, (x, y, w, h), error), error in -1 (left) .. +1 (right).
    """
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

def flash_once():
    """Fire the onboard flash once via GPIO18."""
    if GPIO is None:
        print("[FLASH] (Pi-side, no GPIO — dev machine)")
        return
    GPIO.output(FLASH_PIN, GPIO.HIGH)
    time.sleep(FLASH_ON_TIME)
    GPIO.output(FLASH_PIN, GPIO.LOW)
    print("[FLASH] (Pi-side)")


def handle_scan_request(request_id, parameter):
    """
    Sample a short burst of frames, vote on the best target, flash if it's
    already centered, and send back exactly one report. `parameter` is decoded
    but unused (reserved by the wire format for future per-scan tuning).
    """
    votes = []
    frames_read = 0
    for _ in range(SCAN_BURST_FRAMES):
        ok, frame = cap.read()
        if not ok:
            continue
        frames_read += 1
        target = pick_target(yolo_detect_all(frame, frame.shape[1]))
        if target is not None:
            votes.append(target)
        publish_frame(frame, target, "SCANNING")

    if frames_read == 0:
        print(f"[SCAN #{request_id}] camera unavailable")
        link.send_pi_report(request_id, PI_ACTION_SCAN_TELETUBBIES, PI_RESULT_CAMERA_FAULT)
        return

    if not votes:
        print(f"[SCAN #{request_id}] no detection")
        link.send_pi_report(request_id, PI_ACTION_SCAN_TELETUBBIES, PI_RESULT_NOT_FOUND)
        return

    tally = Counter(v[0] for v in votes)
    winner, count = tally.most_common(1)[0]
    if count < SCAN_MIN_VOTES:
        print(f"[SCAN #{request_id}] inconsistent ({winner} only {count}/{SCAN_BURST_FRAMES})")
        link.send_pi_report(request_id, PI_ACTION_SCAN_TELETUBBIES, PI_RESULT_NOT_FOUND)
        return

    matching_errors = [v[2] for v in votes if v[0] == winner]
    mean_error = sum(matching_errors) / len(matching_errors)
    confidence_percent = round(100 * count / SCAN_BURST_FRAMES)

    print(f"[SCAN #{request_id}] {winner} err={mean_error:+.3f} conf={confidence_percent}%")

    # The robot is stationary during a scan, so a centered reading means it's
    # pointed at the target right now — flash before replying, not after.
    if abs(mean_error) < ALIGN_THRESHOLD:
        flash_once()
        flash_once()
        flash_once()
        mean_error = 0

    link.send_pi_report(
        request_id, PI_ACTION_SCAN_TELETUBBIES, PI_RESULT_OK,
        target_id=IDENTITY_TO_TARGET_ID[winner],
        horizontal_error=mean_error,
        confidence_percent=confidence_percent)


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

cap = cv2.VideoCapture(CAMERA_INDEX)


def control_loop():
    """Owns the camera. Idles (streaming only) until a PI_REQUEST arrives."""
    consecutive_failures = 0
    MAX_FAILURES = 30        # ADJUST: bad reads in a row before we give up (~1s @30fps)

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
                print("[control_loop] camera unavailable — stopping.")
                stop_event.set()
                break
            time.sleep(0.03)
            continue
        consecutive_failures = 0

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
        <title>Detector — reactive</title>
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
