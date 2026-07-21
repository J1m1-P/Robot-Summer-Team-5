"""
Teletubby Detector — (YOLO-only)
================================================================================
The merge of the continuous and incremental detectors into ONE program. There is
no "mode" flag: the Pi is fully REACTIVE. It looks only while the ESP holds a
look-window open, stops when it sees a tubby, confirms/aligns/flashes, then tells
the ESP to carry on. Which SEARCH STRATEGY runs is therefore decided entirely by
the ESP — the same Pi binary serves both:

  CONTINUOUS : ESP opens the look-window once and tape-follows with it held open
               (re-opens after each RESUME). The Pi looks the whole time.
  INCREMENTAL: ESP brackets each in-place sweep with LOOK_START/LOOK_END and drives
               DARK (window closed) between sweeps. The Pi looks only during sweeps.

The two standalone versions are kept for reference / A-B snapshots:
  teletubby_detector_headless.py  — continuous only (Pi-initiated SCAN/FOLLOW)
  detector_incremental.py         — incremental only (WAIT/LOOK)
This file supersedes both once the ESP-side look-window protocol is settled.

THE STATE MACHINE
--------------------------------------------------------------------------------
  WAIT     Not looking (ESP driving / between sweeps). YOLO doesn't run.
           ESP signals LOOK_START -> LOOK.  ROUTINE_DONE -> DONE(no more sweeps left, but 2 teletubbies still remain).
  LOOK     ESP has the window open; Pi runs YOLO each sampled frame.
           See a tubby -> STOP -> CONFIRM.  ESP signals LOOK_END -> WAIT.
  CONFIRM  Stopped. Vote over frames. Consistent -> ALIGN; false positive -> RESUME -> WAIT.
  ALIGN    Turn to center the tubby, THEN hold centered for SETTLE_FRAMES frames so
           we know the robot actually finished turning and stopped -> FLASH.
           Lost the target -> RESUME -> WAIT.
  FLASH    Robot is settled on the tubby. FLASH IS A PI ACTION (flash_once) — not a
           UART command. Flash, record, TURN(undo the align turn), RESUME -> WAIT.
  DONE     Reached two ways: both tubbies flashed (success), OR the ESP ran out of
           sweeps via ROUTINE_DONE while a tubby was still unfound (gave up). Either
           way: tell the ESP to cancel/stand down its routine and stop looking for
           good. NOTE: the Pi process itself keeps running (still serving the
           stream) — it doesn't exit. If you want it to actually shut down here,
           that needs to be added explicitly (e.g. stop_event.set()).
  flow:  WAIT <-> LOOK -> CONFIRM -> ALIGN -> FLASH -> RESUME -> WAIT ... -> DONE

THE Pi <-> ESP CONTRACT (still to finalize)
--------------------------------------------------------------------------------
  ESP -> Pi (STATUS packets, payload format TBD):
     LOOK_START    open the look-window (begin looking)
     LOOK_END      close it (sweep done)
     ROUTINE_DONE  (optional) ESP ran out of sweeps and we still lack two
  Pi -> ESP (COMMAND packets):
     STOP     freeze to inspect/align      TURN:x  steer (ALIGN only)
     RESUME   continue your routine        DONE    finished — stop everything
  NOTE: no FLASH on the wire — the Pi flashes its own hardware (flash_once). And the
  Pi no longer sends SCAN/FOLLOW; the ESP owns all motion.
"""

import time
import threading
from collections import Counter

import cv2
from flask import Flask, Response, render_template_string
from ultralytics import YOLO
from uart_link import (RobotLink, PACKET_TYPE_STATUS,           # the ESP32 serial link
                       STATUS_LOOK_START, STATUS_LOOK_END, STATUS_ROUTINE_DONE)


# ══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION — everything you tune lives here
# ══════════════════════════════════════════════════════════════════════════════
# ── camera ────────────────────────────────────────────────────────────────────
CAMERA_INDEX = 1          # ADJUST: 0 if one camera, 1 for built-in + USB

# ── model / detection ─────────────────────────────────────────────────────────
MODEL_PATH  = r"E:\runs\detect\train-9\weights\best_ncnn_model"  # ADJUST: .pt or ncnn folder
IMGSZ       = 320         # ADJUST: 320 / 480 / 640 — smaller = faster, less accurate
DETECT_CONF = 0.5         # ADJUST: min YOLO confidence

# ── how often YOLO runs while LOOKing ─────────────────────────────────────────
YOLO_EVERY_N = 2          # ADJUST: run YOLO every Nth frame while LOOKing. CONFIRM/
                          #         ALIGN run every frame. WAIT runs none (not looking).

# ── committing to a target ────────────────────────────────────────────────────
CONFIRM_FRAMES   = 5      # ADJUST: frames to sample while stopped in CONFIRM
CONFIRM_VOTES    = 3      # ADJUST: how many must agree before we commit to ALIGN
ALIGN_THRESHOLD  = 0.08   # ADJUST: |error| below this counts as "centered"
ALIGN_MAX_MISSES = 4      # ADJUST: no-detection frames ALIGN tolerates before giving up
SETTLE_FRAMES    = 3      # ADJUST: consecutive centered frames required before flashing.
                          #         This is the "wait until the robot has turned to its
                          #         place and stopped" guard — raise it if the robot is
                          #         still drifting when the flash fires.
FLASH_COUNT      = 3      # ADJUST: how many flashes once settled
TARGETS_TO_FIND  = 2      # only two tubbies exist and we need both — leave at 2

# ── serial link to the ESP32 ──────────────────────────────────────────────────
SERIAL_PORT = None        # ADJUST: None = DEV MODE (commands just print). "COM5" /
                          #         "/dev/serial0" / "/dev/ttyUSB0" to transmit.
SERIAL_BAUD = 115200      # ADJUST: must match the ESP's baud rate
link = RobotLink(SERIAL_PORT, SERIAL_BAUD) if SERIAL_PORT else None

# ── browser view ──────────────────────────────────────────────────────────────
ENABLE_STREAM = True      # ADJUST: True to watch stream

# ── flash hardware ─────────────────────────────────────────────────────────────
FLASH_PIN = None          # ADJUST: BCM GPIO pin driving the flash (e.g. 18). None =
                          #         DEV MODE (flash_once() just prints, no GPIO needed).
FLASH_ON_SECONDS = 0.15   # ADJUST: how long the pin stays high per flash

if FLASH_PIN is not None:
    import RPi.GPIO as GPIO   
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(FLASH_PIN, GPIO.OUT)
    GPIO.output(FLASH_PIN, GPIO.LOW)


# ══════════════════════════════════════════════════════════════════════════════
# YOLO DETECTION
# ══════════════════════════════════════════════════════════════════════════════
# (Unchanged from teletubby_detector_headless.py — all versions share brains.)

model = YOLO(MODEL_PATH)
YOLO_NAME_MAP = {"DP": "dipsy", "LL": "laa_laa", "PO": "po", "TW": "tinky_winky"}


def yolo_detect(frame, frame_w, exclude=None, want_id=None):
    """
    Run YOLO on one frame and return a SINGLE detection, or None.
    Returns (identity, (x, y, w, h), error). See headless file for full notes.
    """
    exclude = exclude or set()
    results = model(frame, conf=DETECT_CONF, imgsz=IMGSZ, verbose=False)
    boxes = results[0].boxes
    if len(boxes) == 0:
        return None

    xyxy = boxes.xyxy.cpu().numpy()
    cls  = boxes.cls.cpu().numpy().astype(int)
    conf = boxes.conf.cpu().numpy()

    candidates = []
    for i in range(len(boxes)):
        identity = YOLO_NAME_MAP[model.names[cls[i]]]
        if identity in exclude:
            continue
        if want_id is not None and identity != want_id:
            continue
        candidates.append((identity, xyxy[i], conf[i]))

    if not candidates:
        return None

    identity, box, _ = max(candidates, key=lambda c: c[2])   # highest confidence
    x1, y1, x2, y2 = box
    x, y, w, h = int(x1), int(y1), int(x2 - x1), int(y2 - y1)
    error = ((x + w / 2) - frame_w / 2) / (frame_w / 2)
    return identity, (x, y, w, h), error


# ══════════════════════════════════════════════════════════════════════════════
# TALKING TO THE ROBOT
# ══════════════════════════════════════════════════════════════════════════════
#   Command set for the reactive Pi. 
#   STOP    freeze to inspect/align        TURN:x  steer (ALIGN only)
#   RESUME  continue your sweep/drive      DONE    finished teletubby task— stop everything

def send(message):
    """
    Turn a state-machine command string into a real packet. DEV MODE (link None)
    just prints, so the same code runs with or without the ESP attached.
    """
    if link is None:
        print(f"[TX] {message}")
        return
    if message.startswith("TURN:"):
        link.turn(float(message.split(":", 1)[1]))
    elif message == "STOP":     link.stop()
    elif message == "DONE":     link.done()
    elif message == "RESUME":   link.resume()
    else:
        print(f"[TX] unknown command: {message}")


def flash_once():
    """
    Fire the Pi's OWN flash once. Flashing is a Pi action now (not a UART command),
    so its timing is fully under Pi control — we only get here after the robot has
    settled centered on the tubby (see ALIGN's SETTLE_FRAMES guard).
    """
    if FLASH_PIN is None:
        print("[FLASH] (Pi-side, DEV MODE — set FLASH_PIN to drive real hardware)")
        return
    GPIO.output(FLASH_PIN, GPIO.HIGH)
    time.sleep(FLASH_ON_SECONDS)
    GPIO.output(FLASH_PIN, GPIO.LOW)


# ── ESP -> Pi signals ─────────────────────────────────────────────────────────
# Edge events from the ESP. Set by handle_incoming(); consumed (read + cleared)
# via consume() so each event fires exactly once.
esp_look_start   = False   # open the look-window — begin looking
esp_look_end     = False   # close it — sweep finished with nothing
esp_routine_done = False   # (optional) ESP ran all its sweeps, we still lack two


def handle_incoming():
    """
    Drain packets from the ESP and set the matching signal flag. Call once/loop.
    payload[0] is the StatusCode (see robot_common/status_packet.h); payload[1],
    the detail byte, is unused by any of these three signals.
    """
    global esp_look_start, esp_look_end, esp_routine_done
    if link is None:
        return
    for msg_type, payload in link.poll():
        if msg_type != PACKET_TYPE_STATUS or not payload:
            continue
        code = payload[0]
        if code == STATUS_LOOK_START:
            esp_look_start = True
        elif code == STATUS_LOOK_END:
            esp_look_end = True
        elif code == STATUS_ROUTINE_DONE:
            esp_routine_done = True


def consume(flag_name):
    """Read one signal flag and clear it (edge event fires once). Returns bool."""
    g = globals()
    was = g[flag_name]
    g[flag_name] = False
    return was


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
# STATE + CONTROL LOOP
# ══════════════════════════════════════════════════════════════════════════════

WAIT, LOOK, CONFIRM, ALIGN, FLASH, DONE = "WAIT", "LOOK", "CONFIRM", "ALIGN", "FLASH", "DONE"

cap             = cv2.VideoCapture(CAMERA_INDEX)
state           = WAIT     # boot idle; ESP's first LOOK_START -> LOOK
target_id       = None
confirm_votes   = []
confirmed_ids   = set()
visited         = set()
align_error     = None
centered_streak = 0        # consecutive centered frames in ALIGN (the settle counter)
flash_sent      = 0
done_sent       = False
frame_count     = 0
last_detection  = None
align_misses    = 0


def control_loop():
    """Owns the camera and the state machine. Runs on its own thread, forever."""
    global state, target_id, confirm_votes, confirmed_ids, visited
    global align_error, centered_streak, flash_sent, done_sent, frame_count
    global last_detection, align_misses

    consecutive_failures = 0
    MAX_FAILURES = 30        # ADJUST: bad reads in a row before we give up (~1s @30fps)

    while not stop_event.is_set():
        ok, frame = cap.read()
        if not ok:
            consecutive_failures += 1
            print(f"[control_loop] camera read failed "
                  f"({consecutive_failures}/{MAX_FAILURES})")
            if consecutive_failures >= MAX_FAILURES:
                print("[control_loop] camera unavailable — stopping.")
                stop_event.set()
                break
            time.sleep(0.03)
            continue
        consecutive_failures = 0

        frame_w = frame.shape[1]
        frame_count += 1
        handle_incoming()

        # ── Decide whether to run YOLO on this frame ─────────────────────────────
        # Only LOOK/CONFIRM/ALIGN look. WAIT and DONE run NO inference — that's the
        # "stop looking" behaviour and why WAIT is cheap while the ESP drives.
        if state in (CONFIRM, ALIGN):
            want = target_id if state == ALIGN else None
            detection = yolo_detect(frame, frame_w, exclude=visited, want_id=want)
            last_detection = detection
        elif state == LOOK and frame_count % YOLO_EVERY_N == 0:
            last_detection = yolo_detect(frame, frame_w, exclude=visited) ## process frame
            detection = last_detection
        elif state == LOOK:
            detection = last_detection    # skipped frame -> reuse most recent
        else:
            detection = None              # WAIT / DONE -> not looking

        # ── State machine ─────────────────────────────────────────────────────
        if state == DONE:
            if not done_sent:
                send("DONE")              # ESP stops sweeping; we've already stopped looking
                done_sent = True

        # ── WAIT: idle while the ESP drives. Wait for it to open the window. ──
        # No Pi-side kick-off: the ESP owns the routine and self-starts it at
        # power-on (that's the whole point of "reactive" — the Pi never drives).
        # If the ESP needs an explicit go-ahead instead, send it here once, guarded
        # by a one-shot flag.
        elif state == WAIT:
            if consume("esp_look_start"):
                # Discard any stale esp_look_end left over from the PREVIOUS look
                # window. handle_incoming() runs every tick regardless of state, so
                # if the ESP's LOOK_END for that old window arrived while we were
                # off in CONFIRM/ALIGN/FLASH (e.g. we found a tubby right as that
                # sweep ended), it would otherwise sit unconsumed and get misread as
                # "this brand-new window just ended" the instant we start LOOK.
                consume("esp_look_end")
                state = LOOK
            elif consume("esp_routine_done"):
                state = DONE

        # ── LOOK: ESP holds the window open; we watch each sampled frame. ─────
        elif state == LOOK:
            if detection is not None:
                target_id = detection[0]
                confirm_votes = []
                send("STOP")             # freeze so CONFIRM sees a stable frame
                state = CONFIRM
            elif consume("esp_look_end"):
                state = WAIT

        # ── CONFIRM: stopped. Vote across frames. (from headless) ─────────────
        elif state == CONFIRM:
            confirm_votes.append(detection[0] if detection is not None else None)
            if len(confirm_votes) >= CONFIRM_FRAMES:
                tally = Counter(v for v in confirm_votes if v is not None)
                winner, count = tally.most_common(1)[0] if tally else (None, 0)
                confirm_votes = []
                if count >= CONFIRM_VOTES:
                    target_id = winner
                    align_error = None
                    align_misses = 0
                    centered_streak = 0
                    state = ALIGN
                else:
                    target_id = None # False detection
                    send("RESUME")
                    state = WAIT

        # ── ALIGN: center on the tubby, then SETTLE before flashing. ─────────
        elif state == ALIGN:
            if detection is None:
                align_misses += 1
                centered_streak = 0        # lost sight -> not settled
                if align_misses > ALIGN_MAX_MISSES:
                    target_id = None
                    align_error = None
                    align_misses = 0
                    send("RESUME")
                    state = WAIT
            else:
                align_misses = 0
                error = detection[2]
                if align_error is None:
                    align_error = error    # remember the initial offset to undo after FLASH
                if abs(error) < ALIGN_THRESHOLD:
                    # Centered on THIS frame. Don't flash yet — the robot may still be
                    # coasting from the last turn. Insist it STAYS centered for
                    # SETTLE_FRAMES frames = "turned to its place and stopped moving".
                    send("STOP")
                    centered_streak += 1
                    # Deliberately Pi-side only: there's no ESP "turn complete" ack on
                    # the wire, so a vision-side streak is the settle signal for now.
                    # If FLASH ever fires while the robot is visibly still drifting,
                    # that's the sign to raise SETTLE_FRAMES or add a real ESP ack.
                    if centered_streak >= SETTLE_FRAMES:
                        flash_sent = 0
                        centered_streak = 0
                        state = FLASH
                else:
                    centered_streak = 0    # drifted -> reset the settle, keep steering
                    send(f"TURN:{error:.3f}")

        # ── FLASH: settled on the tubby. Pi flashes itself, then resumes. ────
        elif state == FLASH:
            if flash_sent < FLASH_COUNT:
                flash_once()               # Pi-side flash — NOT a UART command
                flash_sent += 1
            else:
                if target_id is not None:
                    confirmed_ids.add(target_id)
                    visited.add(target_id)

                if len(confirmed_ids) >= TARGETS_TO_FIND:
                    state = DONE
                else:
                    # Undo the align turn so the ESP resumes roughly re-aimed, then
                    # hand the routine back and go idle until the next look-window.
                    # NOTE: TURN then RESUME go out back-to-back — if the ESP holds
                    # only one unread packet, space them or fold the turn-back into
                    # RESUME on the firmware side.
                    if align_error is not None:
                        send(f"TURN:{-align_error:.3f}")
                    send("RESUME")
                    state = WAIT

                target_id = None
                align_error = None
                flash_sent = 0

        # ── Publish an annotated frame for viewers (skipped when headless) ────
        if ENABLE_STREAM:
            display = draw_overlay(frame.copy(), detection, state, target_id)
            cv2.putText(display, f"found: {len(confirmed_ids)}/{TARGETS_TO_FIND}",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            ok, buffer = cv2.imencode('.jpg', display)
            if ok:
                frames.publish(buffer.tobytes())

    cap.release()


# ══════════════════════════════════════════════════════════════════════════════
# DISPLAY + WEB SERVER — watch the robot from a browser
# ══════════════════════════════════════════════════════════════════════════════

app = Flask(__name__)


def draw_overlay(frame, detection, state, target_id):
    cv2.putText(frame, f"state: {state}  target: {target_id}", (10, 60),
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
        <h1>Teletubby Detector — UNIFIED REACTIVE (ESP owns strategy, Pi looks on cue)</h1>
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

    if ENABLE_STREAM:
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    else:
        try:
            stop_event.wait()
        except KeyboardInterrupt:
            stop_event.set()
        worker.join(timeout=2.0)