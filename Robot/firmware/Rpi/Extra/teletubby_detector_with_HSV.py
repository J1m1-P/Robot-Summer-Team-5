"""
Teletubby Detector
================================================================================
Goal: drive a robot to find TWO distinct Teletubbies, turn to face each one,
and "flash" at it. A webcam feed is streamed to a browser so you can watch what
the robot sees, with the detection boxes and current state drawn on top.

HOW IT WORKS — two detectors working together
--------------------------------------------------------------------------------
  * HSV colour detection is FAST but dumb: it lights up on anything matching a
    Teletubby's colour, including a same-coloured patch of background. It's the
    trigger — good enough to say "something interesting is over there."
  * A trained YOLO model is SLOWER but smart: it recognises the actual shape of
    a Teletubby and reports its own box and identity. It's the confirmer — it
    only runs after HSV has already stopped the robot, so we pay its cost rarely.

THE STATE MACHINE — the robot is always in exactly one of these states
--------------------------------------------------------------------------------
The ESP owns all driving: it runs the scan sweep and tape-follows the course on
its own. The detector only GATES that motion and steers during alignment.

  SCAN     (start-up) The ESP runs its scan sweep in place; the detector
           watches each frame. Boots into this state. Sees a tubby -> CONFIRM;
           finishes the sweep with nothing -> FOLLOW.
  FOLLOW   The ESP tape-follows the course on its own; the detector watches
           (HSV only) each frame. Sees a tubby -> STOP -> CONFIRM.
  CONFIRM  Stopped. YOLO votes over a few frames. Real tubby -> ALIGN;
           false positive -> back to FOLLOW.
  ALIGN    Turn to center the tubby in the frame. This points the robot OFF the
           tape, so we remember the turn to undo it after flashing.
  FLASH    Centered. Flash, record the tubby, then turn back by -align_error to
           help the ESP re-find the tape, and resume FOLLOW (or DONE).
  DONE     Found the required number of DISTINCT tubbies. Stop for good.

  flow:  SCAN -> CONFIRM -> ALIGN -> FLASH -> FOLLOW -> CONFIRM -> ... -> DONE

COUNTING TWO *DISTINCT* TUBBIES
--------------------------------------------------------------------------------
  `confirmed_ids` is a SET of the identities we've flashed. We're done when it
  holds TARGETS_TO_FIND of them. Because a set can't store the same identity
  twice, we can never miscount the same tubby as two. `visited` is a separate
  set that keeps a finished tubby from being re-detected while we look for the next.
"""

import time
import threading
import cv2
import numpy as np
from collections import Counter
from flask import Flask, Response, render_template_string
from ultralytics import YOLO   # heavy import (pulls in torch) — dev machine for now
from uart_link import RobotLink, PACKET_TYPE_STATUS, STATUS_SCAN_DONE   # the ESP32 serial link


# ══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION — the numbers you tune
# ══════════════════════════════════════════════════════════════════════════════

# Which camera to open (0 if you only have one, 1 if you have a built-in + USB cam).
cap = cv2.VideoCapture(1)

# A colour blob smaller than this many pixels is treated as noise, not a tubby.
# Raise it if speckle triggers false hits; lower it if far-away tubbies are missed.
MIN_CONTOUR_AREA = 500

# HSV colour range for each Teletubby: a (lower, upper) bound pair per colour.
# Tune with hsv_tuner.py at the real venue lighting. dipsy/laa_laa hues overlap,
# but that's OK now — YOLO tells those two apart by shape. `po` stays disabled
# until its range is tuned; while it's off, the detector can't find a po tubby.
COLOR_RANGES = {
    "tinky_winky": [(np.array([127, 24, 23]), np.array([154, 174, 199]))],
    "dipsy":       [(np.array([23, 26, 15]), np.array([73, 195, 207]))],
    "laa_laa":     [(np.array([15, 26, 15]), np.array([57, 195, 207]))],
    # "po":        [(np.array([0, 0, 0]), np.array([0, 0, 0]))],
}

ALIGN_THRESHOLD = 0.08   # |error| below this counts as "centered" (see the error note)
FLASH_COUNT     = 3      # how many flashes once centered
TARGETS_TO_FIND = 2      # how many DISTINCT tubbies to find before stopping

# YOLO confirmation settings:
MODEL_PATH = r"E:\runs\detect\train-9\weights\best_ncnn_model"   # trained yolo model
CONFIRM_CONF   = 0.5     # min YOLO confidence. 0.5–0.6 rejects weak background hits.
CONFIRM_IOU    = 0.3     # min overlap between YOLO's box and HSV's box to trust it
CONFIRM_FRAMES = 5       # frames to sample while stopped in CONFIRM (cheap: robot is stopped)
CONFIRM_VOTES  = 3       # how many of those frames must agree to commit

# Serial link to the ESP32 (see uart_link.py). Leave SERIAL_PORT = None to run in
# DEV MODE — commands just print, no hardware needed. Set it to the real device to
# transmit: "COM5" on Windows, "/dev/serial0" or "/dev/ttyUSB0" on the Pi.
SERIAL_PORT = None
SERIAL_BAUD = 115200     # MUST match UartLinkConfig.baud_rate on the ESP
link = RobotLink(SERIAL_PORT, SERIAL_BAUD) if SERIAL_PORT else None

# Run the browser view (dev machine) or not (the Pi). When False the control loop
# skips the overlay draw AND the JPEG encode entirely — pure telemetry cost the
# robot doesn't need, and CPU it can spend on inference instead. Same file, both
# ways: leave True at your desk, set False on the robot.
ENABLE_STREAM = True


# ══════════════════════════════════════════════════════════════════════════════
# CAMERA VISION — turn a colour into a box and a steering error
# ══════════════════════════════════════════════════════════════════════════════

def build_mask(hsv_frame, ranges):
    """
    Make a black-and-white mask: white where the pixel falls inside any of the
    given (lower, upper) HSV ranges, black elsewhere. Most colours use one range.
    """
    combined = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        combined = cv2.bitwise_or(combined, cv2.inRange(hsv_frame, lower, upper))
    return combined


def find_blob(mask, min_area=MIN_CONTOUR_AREA):
    """
    Find the single biggest white region in the mask that's larger than
    `min_area`. Returns that region's outline (a contour), or None if the only
    white regions are tiny specks of noise.
    """
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < min_area:
        return None
    return largest


def get_bbox_and_error(contour, frame_width):
    """
    Turn a blob outline into (x, y, w, h, error):
      * (x, y, w, h) is the upright bounding box (x,y = top-left corner).
      * `error` is how far the box's center is from the frame's center,
        normalised to [-1, +1]:  -1 = target at far left, 0 = centered,
        +1 = far right. This is the steering signal the robot turns on.
    """
    x, y, w, h = cv2.boundingRect(contour)
    box_center_x = x + w / 2
    frame_center_x = frame_width / 2
    error = (box_center_x - frame_center_x) / frame_center_x
    return x, y, w, h, error


def scan_all(hsv_frame, frame_width, exclude=None):
    """
    Look for EVERY colour in COLOR_RANGES (skipping any in `exclude`) and return
    a dict of name -> (x, y, w, h, error) for each colour visible right now.
    Used by SCAN and FOLLOW to spot a candidate.
    """
    exclude = exclude or set()
    found = {}
    for name, ranges in COLOR_RANGES.items():
        if name in exclude:
            continue
        contour = find_blob(build_mask(hsv_frame, ranges))
        if contour is not None:
            found[name] = get_bbox_and_error(contour, frame_width)
    return found


# ══════════════════════════════════════════════════════════════════════════════
# YOLO CONFIRMATION — a smarter second opinion
# ══════════════════════════════════════════════════════════════════════════════

# Load the trained model ONCE (reloading weights every frame would be very slow).
model = YOLO(MODEL_PATH)

# YOLO reports abbreviated class names; map them to the COLOR_RANGES keys so the
# same identity string is used everywhere. (model.names == {0:'DP',1:'LL',...}.)
YOLO_NAME_MAP = {"DP": "dipsy", "LL": "laa_laa", "PO": "po", "TW": "tinky_winky"}


def iou(box_a, box_b):
    """
    Intersection-over-Union of two boxes given as (x1, y1, x2, y2) corners.
    It's the overlap area divided by the combined area: 0.0 = no overlap,
    1.0 = identical. We use it to check YOLO's box and HSV's box describe the
    SAME object (both point at the same spot), not two things across the room.
    """
    inter_x1 = max(box_a[0], box_b[0])
    inter_y1 = max(box_a[1], box_b[1])
    inter_x2 = min(box_a[2], box_b[2])
    inter_y2 = min(box_a[3], box_b[3])
    inter_w = max(0, inter_x2 - inter_x1)
    inter_h = max(0, inter_y2 - inter_y1)
    intersection = inter_w * inter_h

    area_a = (box_a[2] - box_a[0]) * (box_a[3] - box_a[1])
    area_b = (box_b[2] - box_b[0]) * (box_b[3] - box_b[1])
    union = area_a + area_b - intersection
    if union == 0:
        return 0.0
    return intersection / union


def yolo_confirm(frame, hsv_bbox, conf=CONFIRM_CONF, min_iou=CONFIRM_IOU):
    """
    Ask YOLO for a SECOND opinion on one frame, and return the tubby's identity
    if it agrees with HSV — otherwise None (a vote against).

    Steps:
      1. Run YOLO on the raw BGR frame (ultralytics takes BGR directly).
      2. Of all YOLO's boxes, pick the one overlapping HSV's box the most.
      3. If that overlap clears `min_iou`, return that box's identity (mapped to
         our naming). We trust YOLO's identity over HSV's colour guess — that's
         how dipsy and laa_laa get told apart despite their overlapping hues.
      4. No boxes, or nothing overlaps enough -> None.
    """
    x, y, w, h = hsv_bbox
    hsv_corners = (x, y, x + w, y + h)

    results = model(frame, conf=conf, verbose=False)
    yolo_boxes = results[0].boxes.xyxy.cpu().numpy()          # (N, 4) corner boxes
    yolo_cls   = results[0].boxes.cls.cpu().numpy().astype(int)  # (N,) class indices
    if len(yolo_boxes) == 0:
        return None

    ious = [iou(box, hsv_corners) for box in yolo_boxes]
    best = int(np.argmax(ious))
    if ious[best] < min_iou:
        return None
    return YOLO_NAME_MAP[model.names[yolo_cls[best]]]


# ══════════════════════════════════════════════════════════════════════════════
# TALKING TO THE ROBOT
# ══════════════════════════════════════════════════════════════════════════════
# The ESP owns motion. The detector only gates it with these commands:
#   SCAN    start the initial scan sweep    FOLLOW  drive/tape-follow the course
#   STOP    halt                            TURN:x  steer (used only in ALIGN)
#   FLASH   flash at a tubby                DONE    finished

def send(message):
    """
    Translate a state-machine command string into a real packet on the wire.
    In DEV MODE (link is None) it just prints, so the exact same detector code
    runs with or without the ESP connected.
    """
    if link is None:
        print(f"[TX] {message}")
        return
    if message.startswith("TURN:"):
        link.turn(float(message.split(":", 1)[1]))
    elif message == "STOP":     link.stop()
    elif message == "FOLLOW":   link.follow()
    elif message == "SCAN":     link.scan()
    elif message == "FLASH":    link.flash()
    elif message == "DONE":     link.done()
    else:
        print(f"[TX] unknown command: {message}")


# Set True by handle_incoming() when the ESP reports its scan sweep is complete.
esp_scan_done = False


def handle_incoming():
    """
    Drain any packets the ESP has sent us and react. Call once per loop. For now
    the only thing we need from the ESP is 'the scan sweep finished'.
    """
    global esp_scan_done
    if link is None:
        return
    for msg_type, payload in link.poll():
        if msg_type == PACKET_TYPE_STATUS and payload and payload[0] == STATUS_SCAN_DONE:
            esp_scan_done = True


def esp_scan_finished():
    """True once the ESP has reported the initial scan sweep is complete."""
    return esp_scan_done


# ══════════════════════════════════════════════════════════════════════════════
# FRAME HAND-OFF — the one shared thing the two threads pass between them
# ══════════════════════════════════════════════════════════════════════════════
# The control thread owns the camera and the state machine; Flask only ever reads
# the LAST annotated frame the control thread produced. That single hand-off is
# the only place the two threads touch, so it's the only place that needs a lock.
# (The state-machine globals below don't: the control thread is their sole writer,
# and Flask reading a slightly stale `state` string for display text is harmless.)

class FrameBuffer:
    """
    A one-slot, thread-safe mailbox for the most recent JPEG frame. "Latest wins":
    a browser always sees the freshest frame and never has to work through a
    backlog. Any number of stream consumers can read from it at once.
    """
    def __init__(self):
        # A Condition is a lock plus a wait/notify channel. The lock guards
        # _jpeg/_seq against half-written reads; the wait/notify lets a consumer
        # sleep until a new frame lands instead of busy-polling at 100% CPU.
        self._cond = threading.Condition()
        self._jpeg = None   # newest JPEG bytes, or None until the first frame lands
        self._seq  = 0      # bumped every publish so a reader can tell a NEW frame
                            # from the one it already sent

    def publish(self, jpeg_bytes):
        """Store the newest frame and wake every waiting consumer. Called by the
        control thread once per frame (only while streaming is enabled)."""
        with self._cond:                 # acquire the lock
            self._jpeg = jpeg_bytes
            self._seq += 1
            self._cond.notify_all()      # wake anyone blocked in get_newer_than()

    def get_newer_than(self, last_seq, timeout=1.0):
        """
        Block until a frame newer than `last_seq` is available (or `timeout`
        seconds elapse), then return (jpeg_bytes, current_seq).

        On timeout return (None, last_seq) so the caller can loop and keep its
        HTTP connection alive rather than hanging forever when the control loop
        hasn't produced its first frame yet or has momentarily stalled.
        """
        with self._cond:
            # wait_for() releases the lock while it sleeps and re-takes it on wake.
            # It returns True once the predicate holds, or False if it timed out.
            got_new = self._cond.wait_for(lambda: self._seq != last_seq, timeout)
            if not got_new:
                return None, last_seq
            return self._jpeg, self._seq


frames = FrameBuffer()          # the shared mailbox between control loop and Flask
stop_event = threading.Event()  # set this to ask the control loop to exit cleanly


# ══════════════════════════════════════════════════════════════════════════════
# THE STATE MACHINE — one step per camera frame
# ══════════════════════════════════════════════════════════════════════════════

SCAN, FOLLOW, CONFIRM, ALIGN, FLASH, DONE = "SCAN", "FOLLOW", "CONFIRM", "ALIGN", "FLASH", "DONE"

# Everything the state machine remembers between frames. The control loop is the
# only writer of these, so they need no lock (see the frame hand-off note above).
state         = SCAN     # robot boots here: the initial scan sweep
target_name   = None     # HSV colour of the current candidate (drives steering)
confirmed_id  = None     # YOLO identity of the current candidate (drives counting)
confirm_votes = []       # identity (or None) collected each frame during CONFIRM
confirmed_ids = set()    # DISTINCT tubbies flashed so far — reaching TARGETS_TO_FIND = done
visited       = set()    # colour/identity handles excluded from future detection
align_error   = None     # steering error captured when ALIGN starts; undone after FLASH
scan_started  = False    # so the one-shot SCAN command fires only once
flash_sent    = 0
done_sent     = False     # so DONE only announces itself once


def control_loop():
    """
    The sole owner of the camera and the state machine. Runs on its OWN thread
    from start-up and keeps stepping the machine every frame whether or not a
    browser is connected — that independence is the whole point of the decoupling.
    When streaming is enabled it also publishes an annotated JPEG for viewers.
    """
    global state, target_name, confirmed_id, confirm_votes
    global confirmed_ids, visited, align_error, scan_started, flash_sent, done_sent

    consecutive_failures = 0        # bad cap.read()s in a row
    MAX_FAILURES = 30               # ~1 s at 30 fps of solid failure before we bail

    # stop_event lets __main__ (or the failure path below) shut the loop down.
    while not stop_event.is_set():
        ok, frame = cap.read()
        if not ok:
            # In the old single-threaded code a bad read just `break`ed the
            # generator, and the browser noticed the dead stream. Here the loop is
            # a background thread, so a silent death would be invisible. Policy:
            # tolerate brief hiccups (USB camera re-enumerating, a dropped frame),
            # but if the camera is truly gone, stop loudly. Tune MAX_FAILURES.
            consecutive_failures += 1
            print(f"[control_loop] camera read failed "
                  f"({consecutive_failures}/{MAX_FAILURES})")
            if consecutive_failures >= MAX_FAILURES:
                print("[control_loop] camera unavailable — stopping.")
                stop_event.set()
                break
            time.sleep(0.03)        # ~30 ms breather before retrying
            continue
        consecutive_failures = 0    # a good frame resets the streak

        frame_w = frame.shape[1]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        current_bbox_error = None   # what to draw this frame (None = nothing)

        handle_incoming()           # react to anything the ESP sent us

        # ── DONE: finished. Announce once, then idle. ──────────────────────────
        if state == DONE:
            if not done_sent:
                send("DONE")
                done_sent = True

        # ── SCAN: start-up scan sweep. ESP moves; we only watch. ────────────────
        elif state == SCAN:
            if not scan_started:
                send("SCAN")                            # trigger the ESP's scan sweep, once
                scan_started = True
            detections = scan_all(hsv, frame_w, exclude=visited)
            if detections:
                target_name = next(iter(detections))
                confirm_votes = []
                send("STOP")
                state = CONFIRM
            elif esp_scan_finished():                   # scan done, nothing found
                send("FOLLOW")                          # start tape-following the course
                state = FOLLOW
            # else: ESP still running its scan sweep — keep watching.

        # ── FOLLOW: ESP tape-follows on its own; we watch for a tubby. ─────────
        elif state == FOLLOW:
            detections = scan_all(hsv, frame_w, exclude=visited)
            if detections:
                target_name = next(iter(detections))
                confirm_votes = []
                send("STOP")                            # interrupt the drive to take a look
                state = CONFIRM
            # else: nothing yet — let the ESP keep following (no command needed).

        # ── CONFIRM: stopped. Vote with YOLO over several frames. ──────────────
        elif state == CONFIRM:
            # Re-find the colour on THIS frame so HSV's box and YOLO's box come
            # from the same image, then collect one vote (an identity, or None).
            contour = find_blob(build_mask(hsv, COLOR_RANGES[target_name]))
            if contour is not None:
                current_bbox_error = get_bbox_and_error(contour, frame_w)
                x, y, w, h, _ = current_bbox_error
                confirm_votes.append(yolo_confirm(frame, (x, y, w, h)))
            else:
                confirm_votes.append(None)              # HSV lost it this frame -> vote against

            if len(confirm_votes) >= CONFIRM_FRAMES:
                # Enough samples collected — tally the (non-None) votes.
                tally = Counter(v for v in confirm_votes if v is not None)
                winner, count = tally.most_common(1)[0] if tally else (None, 0)
                confirm_votes = []
                if count >= CONFIRM_VOTES:
                    confirmed_id = winner               # YOLO's verdict on identity
                    state = ALIGN
                else:
                    # Not enough agreement -> false positive. Resume following;
                    # driving on changes the view, so the patch won't linger.
                    target_name = confirmed_id = None
                    send("FOLLOW")
                    state = FOLLOW
            # else: still sampling — stay in CONFIRM, robot stays stopped.

        # ── ALIGN: real tubby. Turn until centered (points us off the tape). ───
        elif state == ALIGN:
            contour = find_blob(build_mask(hsv, COLOR_RANGES[target_name]))
            if contour is None:
                # Lost sight mid-align -> hand back to the ESP to re-find the tape.
                target_name = confirmed_id = None
                align_error = None
                send("FOLLOW")
                state = FOLLOW
            else:
                current_bbox_error = get_bbox_and_error(contour, frame_w)
                error = current_bbox_error[4]
                if align_error is None:
                    align_error = error                 # remember the initial offset to undo later
                if abs(error) < ALIGN_THRESHOLD:
                    send("STOP")                        # centered
                    flash_sent = 0
                    state = FLASH
                else:
                    send(f"TURN:{error:.3f}")           # steer proportionally to the error

        # ── FLASH: centered. Flash, record, turn back, resume driving. ─────────
        elif state == FLASH:
            if flash_sent < FLASH_COUNT:
                send("FLASH")
                flash_sent += 1
            else:
                # Record this tubby (distinctness ledger) and exclude BOTH its
                # handles so overlapping colours can't re-detect it.
                if confirmed_id is not None:
                    confirmed_ids.add(confirmed_id)
                    visited.add(confirmed_id)
                visited.add(target_name)

                if len(confirmed_ids) >= TARGETS_TO_FIND:
                    state = DONE
                else:
                    # Undo the align turn (turn by -error) so the ESP starts
                    # roughly re-aimed at the tape, then hand driving back to it.
                    # NOTE: these two commands go out back-to-back — if the ESP
                    # can only hold one unread packet, space them out or fold the
                    # turn-back into a single "resume" command on the firmware side.
                    if align_error is not None:
                        send(f"TURN:{-align_error:.3f}")
                    send("FOLLOW")
                    state = FOLLOW

                target_name = confirmed_id = None
                align_error = None
                flash_sent = 0

        # ── Publish an annotated frame for browser viewers. ────────────────────
        # Skipped WHOLESALE when headless: no overlay draw, no JPEG encode. On the
        # Pi that's real CPU handed back to inference. The state machine above has
        # already done its work by this point, so skipping this changes nothing
        # about the robot's behaviour — only whether anyone can watch.
        if ENABLE_STREAM:
            display = draw_overlay(frame.copy(), current_bbox_error, state, target_name)
            cv2.putText(display, f"found: {len(confirmed_ids)}/{TARGETS_TO_FIND}",
                        (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            ok, buffer = cv2.imencode('.jpg', display)
            if ok:
                frames.publish(buffer.tobytes())   # hand it to the mailbox; Flask reads it

    # Loop exited (stop_event set or camera lost): release the device so a restart
    # can re-open it cleanly.
    cap.release()


def generate_frames():
    """
    Passive consumer for the browser stream. It NO LONGER touches the camera or
    the state machine — it just serves whatever annotated frame the control thread
    last published, blocking until a newer one is ready. Because it's now passive,
    the state machine runs at full speed even with zero browsers connected.
    """
    last_seq = 0
    while True:
        jpeg, last_seq = frames.get_newer_than(last_seq, timeout=1.0)
        if jpeg is None:
            continue    # no new frame yet (start-up or a stall) — keep conn alive
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + jpeg + b'\r\n')


# ══════════════════════════════════════════════════════════════════════════════
# DISPLAY + WEB SERVER — watch the robot from a browser
# ══════════════════════════════════════════════════════════════════════════════

app = Flask(__name__)


def draw_overlay(frame, bbox_error, state, target_name):
    """Draw the current state, and (if we have one) the tubby's box, center dot,
    the frame's center line, and the steering error."""
    cv2.putText(frame, f"state: {state}  target: {target_name}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    if bbox_error is None:
        cv2.putText(frame, "no blob", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return frame

    x, y, w, h, error = bbox_error
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)       # the box
    cv2.circle(frame, (x + w // 2, y + h // 2), 4, (0, 0, 255), -1)     # its center
    cx = frame.shape[1] // 2
    cv2.line(frame, (cx, 0), (cx, frame.shape[0]), (255, 255, 0), 1)    # frame center line
    cv2.putText(frame, f"err: {error:+.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return frame


@app.route('/')
def index():
    return render_template_string('''
    <html>
    <head>
        <title>Detector</title>
        <style>
            body { font-family: monospace; background: #1a1a1a; color: #eee; padding: 20px; }
            img  { display: block; width: 640px; border: 1px solid #444; }
        </style>
    </head>
    <body>
        <h1>Teletubby Detector — HSV trigger + YOLO confirm</h1>
        <img src="/stream">
    </body>
    </html>
    ''')


@app.route('/stream')
def stream():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    # Start the state machine on its own thread FIRST, so it runs with or without
    # a browser attached. daemon=True means it won't keep the process alive on its
    # own — it dies when main exits.
    worker = threading.Thread(target=control_loop, daemon=True)
    worker.start()

    if ENABLE_STREAM:
        # threaded=True is REQUIRED: a browser holding '/stream' open ties up a
        # worker, and the default single-threaded dev server would then be unable
        # to serve '/' at the same time. debug must stay False — the debug
        # reloader forks a SECOND process, which would open the camera and run the
        # state machine TWICE. Both are load-bearing here, not style choices.
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    else:
        # Headless (the Pi): no web server at all. Keep the main thread alive until
        # the control loop asks to stop, and shut down cleanly on Ctrl-C.
        try:
            stop_event.wait()        # block here while the daemon loop does the work
        except KeyboardInterrupt:
            stop_event.set()         # Ctrl-C: ask the loop to exit...
        worker.join(timeout=2.0)     # ...and give it a moment to release the camera