"""
Teletubby Detector — CONTINUOUS SERVO (YOLO-only)
================================================================================
A variant of detector_reactive.py where centering is a CONTINUOUS closed-loop
servo instead of a stop-and-pivot. In the reactive/incremental files the robot
FREEZES (STOP) and fires discrete TURN:x nudges until |error| < threshold. Here
the robot keeps MOVING and the Pi streams a steering correction (STEER:x) every
frame, so the heading converges while the robot is still driving. Only once the
target is centered AND settled do we STOP, then flash.

WHAT CHANGED vs detector_reactive.py
--------------------------------------------------------------------------------
  * ALIGN (stop, then pivot) -> TRACK (drive, stream STEER continuously).
  * New command STEER:x = "bias your heading by x WHILE you keep driving" — a
    different meaning from TURN:x (pivot in place). New opcode, see uart_link TODO.
  * A real controller (SteerController, PD) sits between error and command. Raw
    error-as-command works when stopped but oscillates on the move.
  * "Centered" = |error| < threshold HELD for SETTLE_FRAMES frames (a deadband the
    loop converges into), THEN STOP, THEN flash. It is never exactly zero live.
  * YOLO runs EVERY frame in TRACK (no YOLO_EVERY_N skipping) — stale error on a
    moving robot = overshoot.
  * The post-flash "undo the align turn" (TURN:-align_error) is GONE — in a servo
    the robot arced toward the target as it drove; there is no single pivot to undo.

TWO TARGETS (unchanged mechanism)
--------------------------------------------------------------------------------
  * `visited` (flashed tubbies) is passed as exclude= to yolo_detect, so a tubby
    you've already flashed can't be re-selected.
  * `want_id=target_id` in CONFIRM/TRACK locks the controller to the ONE committed
    target even when both tubbies are in frame — otherwise a "highest confidence"
    pick could flip between them frame-to-frame and the robot would wobble.
  * After flashing A: visited={A} -> RESUME -> the next LOOK/CONFIRM can only pick
    B. confirmed_ids == 2 -> DONE.

THE STATE MACHINE
--------------------------------------------------------------------------------
  WAIT     Not looking (ESP driving / between sweeps). YOLO doesn't run.
           ESP signals LOOK_START -> LOOK.  ROUTINE_DONE -> DONE.
  LOOK     ESP has the window open; Pi runs YOLO each sampled frame.
           See a tubby -> commit it -> CONFIRM (robot keeps moving).
           ESP signals LOOK_END -> WAIT.
  CONFIRM  Vote over frames (on the move) to reject false positives.
           Consistent -> TRACK; false positive -> RESUME -> WAIT.
  TRACK    Continuous servo: stream STEER:controller(error) every frame while the
           robot drives, until the target is centered and held for SETTLE_FRAMES.
           Then STOP -> FLASH.  Lost the target for too long -> RESUME -> WAIT.
  FLASH    Stopped and settled on the tubby. Pi flashes itself (flash_once), records
           the target, RESUME -> WAIT.
  DONE     Both tubbies flashed. Tell the ESP to stop; stop looking.

  flow:  WAIT <-> LOOK -> CONFIRM -> TRACK -> FLASH -> RESUME -> WAIT ... -> DONE

THE Pi <-> ESP CONTRACT (still to finalize with the teammate)
--------------------------------------------------------------------------------
  ESP -> Pi (STATUS packets, payload format TBD):
     LOOK_START / LOOK_END / ROUTINE_DONE   (same as detector_reactive.py)
  Pi -> ESP (COMMAND packets):
     STEER:x  continuous heading bias while driving   (NEW — needs CMD_STEER)
     STOP     halt (only sent once centered+settled)
     RESUME   continue your routine                   DONE  finished — stop
  NOTE: no FLASH on the wire — the Pi flashes its own hardware (flash_once).
"""

import time
import threading
from collections import Counter

import cv2
from flask import Flask, Response, render_template_string
from ultralytics import YOLO
from uart_link import RobotLink, PACKET_TYPE_STATUS   # the ESP32 serial link


# ══════════════════════════════════════════════════════════════════════════════
# CONFIGURATION — everything you tune lives here
# ══════════════════════════════════════════════════════════════════════════════
# ── camera ────────────────────────────────────────────────────────────────────
CAMERA_INDEX = 1          # ADJUST: 0 if one camera, 1 for built-in + USB

# ── model / detection ─────────────────────────────────────────────────────────
MODEL_PATH  = r"E:\runs\detect\train-9\weights\best_ncnn_model"  # ADJUST: .pt or ncnn folder
IMGSZ       = 640         # ADJUST: 320 / 480 / 640 — smaller = faster, less accurate
DETECT_CONF = 0.5         # ADJUST: min YOLO confidence

# ── how often YOLO runs while LOOKing ─────────────────────────────────────────
YOLO_EVERY_N = 2          # ADJUST: run YOLO every Nth frame while LOOKing (cheap search).
                          #         CONFIRM/TRACK run EVERY frame — the servo needs the
                          #         freshest error or it overshoots. WAIT runs none.

# ── committing to a target ────────────────────────────────────────────────────
CONFIRM_FRAMES   = 5      # ADJUST: frames to sample while voting in CONFIRM
CONFIRM_VOTES    = 3      # ADJUST: how many must agree before we commit to TRACK

# ── the servo (TRACK) ─────────────────────────────────────────────────────────
STEER_KP        = 0.6     # ADJUST: proportional gain. Too high -> oscillates; too low
                          #         -> sluggish. Start low and raise until it just starts
                          #         to overshoot, then back off ~20%.
STEER_KD        = 0.15    # ADJUST: derivative gain — damps overshoot from loop latency.
                          #         Raise if it oscillates at a fixed Kp; 0 to disable.
STEER_DEADBAND  = 0.03    # ADJUST: |error| below this -> output 0 (don't chase box jitter)
STEER_OUT_LIMIT = 1.0     # ADJUST: clamp on the steer command so one bad frame can't
                          #         slam the wheels. Matches the -1..+1 the ESP expects.
ALIGN_THRESHOLD = 0.08    # ADJUST: |error| below this counts as "centered"
MISS_GRACE      = 2       # ADJUST: no-detection frames to COAST through (YOLO flicker /
                          #         motion blur) before treating the target as lost.
COAST_DECAY     = 0.6     # ADJUST: shrink the held steer each coasted frame (<1) so a
                          #         real loss doesn't keep turning hard toward a ghost.
ALIGN_MAX_MISSES = 6      # ADJUST: no-detection frames TRACK tolerates before giving up
SETTLE_FRAMES   = 3       # ADJUST: consecutive centered frames (after STOP) required
                          #         before flashing = "robot has actually stopped centered"
FLASH_COUNT     = 3       # ADJUST: how many flashes once settled
TARGETS_TO_FIND = 2       # only two tubbies exist and we need both — leave at 2

# ── serial link to the ESP32 ──────────────────────────────────────────────────
SERIAL_PORT = None        # ADJUST: None = DEV MODE (commands just print). "COM5" /
                          #         "/dev/serial0" / "/dev/ttyUSB0" to transmit.
SERIAL_BAUD = 115200      # ADJUST: must match the ESP's baud rate
link = RobotLink(SERIAL_PORT, SERIAL_BAUD) if SERIAL_PORT else None

# ── browser view ──────────────────────────────────────────────────────────────
ENABLE_STREAM = True      # ADJUST: True to watch (dev). False on the robot.


# ══════════════════════════════════════════════════════════════════════════════
# YOLO DETECTION
# ══════════════════════════════════════════════════════════════════════════════
# (Unchanged from teletubby_detector_headless.py — all versions share brains.)

model = YOLO(MODEL_PATH)
YOLO_NAME_MAP = {"DP": "dipsy", "LL": "laa_laa", "PO": "po", "TW": "tinky_winky"}


def yolo_detect_all(frame, frame_w, exclude=None, want_id=None):
    """
    Run YOLO on one frame and return ALL selectable detections, highest-confidence
    first. Each is (identity, (x, y, w, h), error) with error in -1 (left) .. +1 (right).

    `exclude`  : identities to drop (we pass `visited` = already-flashed tubbies).
    `want_id`  : if set, keep only this identity. TRACK/CONFIRM pass the committed
                 target so a SECOND tubby in frame can't steal the servo.

    Returning the whole list (not just the best box) is what lets LOOK see BOTH
    tubbies at once and choose which to service first — see pick_target().
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
        if want_id is not None and identity != want_id:
            continue
        x1, y1, x2, y2 = xyxy[i]
        x, y, w, h = int(x1), int(y1), int(x2 - x1), int(y2 - y1)
        error = ((x + w / 2) - frame_w / 2) / (frame_w / 2)
        dets.append((identity, (x, y, w, h), error))
    return dets


def yolo_detect(frame, frame_w, exclude=None, want_id=None):
    """Single best (highest-confidence) selectable detection, or None. Thin wrapper
    over yolo_detect_all — used by CONFIRM/TRACK where want_id has already narrowed
    the field to the one committed target."""
    dets = yolo_detect_all(frame, frame_w, exclude, want_id)
    return dets[0] if dets else None


def pick_target(detections):
    """
    Choose which tubby to service FIRST and note where the OTHER one is.
      detections : list from yolo_detect_all (already excludes visited tubbies).
    Returns (target, other_side):
      target      the chosen detection tuple, or None if the list is empty.
      other_side  which way the next tubby sits: -1 left, +1 right, None if alone.

    Policy = closest-to-center first. If both are on the SAME side, servicing the
    nearer one and then continuing the same way keeps the far one in view. If they
    are on OPPOSITE sides, other_side tells us which way to turn BACK after the flash
    (we'll lose the far one from frame during the first turn — this makes the
    re-acquisition directed instead of a blind sweep).
    """
    if not detections:
        return None, None
    ordered = sorted(detections, key=lambda d: abs(d[2]))   # nearest center first
    target = ordered[0]
    other_side = None
    if len(ordered) > 1:
        other_side = -1 if ordered[1][2] < 0 else 1
    return target, other_side


# ══════════════════════════════════════════════════════════════════════════════
# THE STEERING CONTROLLER — turns a stream of errors into a stream of commands
# ══════════════════════════════════════════════════════════════════════════════
# This is the heart of the servo. It lives on the Pi (not the ESP) so you can tune
# the gains live in Python without reflashing firmware. The cost is that UART
# latency sits inside the loop — keep packets small and send at frame rate.

class SteerController:
    """
    PD controller on the horizontal error.
      error : normalized -1..+1 (left..right of center), from yolo_detect.
      out   : steering correction in the same -1..+1 range the ESP expects.
    """

    def __init__(self, kp, kd, deadband, out_limit):
        self.kp, self.kd = kp, kd
        self.deadband  = deadband
        self.out_limit = out_limit
        self._prev_error = 0.0
        self._prev_t     = None

    def update(self, error, now):
        """Return the steer command for this frame's `error` at time `now` (seconds)."""
        # Deadband: inside it we're centered enough — don't chase box jitter, and
        # forget history so the D term doesn't kick when we drift back out.
        if abs(error) < self.deadband:
            self.reset()
            return 0.0

        # Derivative term. The first frame after a reset has no previous timestamp,
        # so there's no dt yet — skip D that frame. Guard dt <= 0 against a stalled
        # or non-monotonic clock so we never divide by zero.
        if self._prev_t is None:
            derivative = 0.0
        else:
            dt = now - self._prev_t
            derivative = (error - self._prev_error) / dt if dt > 0 else 0.0

        out = self.kp * error + self.kd * derivative
        out = max(-self.out_limit, min(self.out_limit, out))   # clamp both ways

        self._prev_error = error
        self._prev_t = now
        return out

    def reset(self):
        """Forget history — call when we commit to a new target or lose the current one."""
        self._prev_error = 0.0
        self._prev_t     = None


controller = SteerController(STEER_KP, STEER_KD, STEER_DEADBAND, STEER_OUT_LIMIT)


# ══════════════════════════════════════════════════════════════════════════════
# TALKING TO THE ROBOT
# ══════════════════════════════════════════════════════════════════════════════
# Command set for the servo Pi. NO FLASH (the Pi flashes itself, see flash_once).
#   STEER:x  continuous heading bias while driving   STOP    halt (once centered)
#   RESUME   continue your sweep/drive routine       DONE    finished — stop everything

def send(message):
    """
    Turn a state-machine command string into a real packet. DEV MODE (link None)
    just prints, so the same code runs with or without the ESP attached.
    """
    if link is None:
        print(f"[TX] {message}")
        return
    if message.startswith("STEER:"):
        # CMD_STEER: continuous heading bias while driving. FIRMWARE MUST decode this
        # as "add to drive heading", NOT "pivot in place" (that's CMD_TURN).
        link.steer(float(message.split(":", 1)[1]))
    elif message == "STOP":     link.stop()
    elif message == "DONE":     link.done()
    elif message.startswith("RESUME"):
        # "RESUME" or "RESUME:L" / "RESUME:R". The suffix is a search-direction hint
        # for the turn-back to the other tubby; firmware may ignore it and plain-resume.
        parts = message.split(":")
        direction = -1.0 if parts[1:] == ["L"] else (1.0 if parts[1:] == ["R"] else 0.0)
        link.resume(direction)
    else:
        print(f"[TX] unknown command: {message}")


def flash_once():
    """
    Fire the Pi's OWN flash once. Flashing is a Pi action (not a UART command), so
    its timing is fully under Pi control — we only get here after the robot has
    STOPPED and settled centered on the tubby (see TRACK's centered->STOP->settle path).
    """
    # TODO: drive the real flash hardware — e.g. set a GPIO pin high, brief sleep,
    # low; or pulse an LED. The dev machine has no GPIO, so it just logs.
    print("[FLASH] (Pi-side)")


# ── ESP -> Pi signals ─────────────────────────────────────────────────────────
# Edge events from the ESP. Set by handle_incoming(); consumed (read + cleared)
# via consume() so each event fires exactly once.
esp_look_start   = False   # open the look-window — begin looking
esp_look_end     = False   # close it — sweep finished with nothing
esp_routine_done = False   # (optional) ESP ran all its sweeps, we still lack two


def handle_incoming():
    """
    Drain packets from the ESP and set the matching signal flag. Call once/loop.

    TODO (needs the teammate's STATUS payload format — same open item as the COMMAND
    sub-format): decode WHICH signal this packet carries and set only that flag. The
    placeholder can't tell them apart, so it just trips LOOK_START — REPLACE it before
    hardware, or the Pi will look at the wrong times.
    """
    global esp_look_start, esp_look_end, esp_routine_done
    if link is None:
        return
    for msg_type, payload in link.poll():
        if msg_type == PACKET_TYPE_STATUS:
            # TODO: inspect `payload` and set exactly one of:
            #   esp_look_start   = True
            #   esp_look_end     = True
            #   esp_routine_done = True
            esp_look_start = True


def consume(flag_name):
    """Read one signal flag and clear it (edge event fires once). Returns bool."""
    g = globals()
    was = g[flag_name]
    g[flag_name] = False
    return was


# ══════════════════════════════════════════════════════════════════════════════
# FRAME HAND-OFF — the one thing the control thread and Flask share
# ══════════════════════════════════════════════════════════════════════════════
# (Unchanged from the other detectors.)

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

WAIT, LOOK, CONFIRM, TRACK, FLASH, DONE = "WAIT", "LOOK", "CONFIRM", "TRACK", "FLASH", "DONE"

cap             = cv2.VideoCapture(CAMERA_INDEX)
state           = WAIT     # boot idle; ESP's first LOOK_START -> LOOK
target_id       = None
confirm_votes   = []
confirmed_ids   = set()
visited         = set()    # tubbies already flashed -> excluded from selection
centered_streak = 0        # consecutive centered frames AFTER stopping (the settle counter)
flash_sent      = 0
done_sent       = False
frame_count     = 0
last_detection  = None
last_look_dets  = []       # most recent multi-detect list (reused on skipped LOOK frames)
last_steer      = 0.0      # last steer command sent (coasted on brief TRACK dropouts)
pending_other_side = None  # where the OTHER tubby was when we committed (-1/+1/None)
track_misses    = 0
start_sent      = False    # one-shot kick-off (see WAIT)


def control_loop():
    """Owns the camera and the state machine. Runs on its own thread, forever."""
    global state, target_id, confirm_votes, confirmed_ids, visited
    global centered_streak, flash_sent, done_sent, frame_count
    global last_detection, last_look_dets, last_steer, pending_other_side
    global track_misses, start_sent

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

        # ── Decide whether to run YOLO this frame ─────────────────────────────
        # CONFIRM/TRACK run EVERY frame (the servo needs fresh error) and are already
        # narrowed to the committed target via want_id=target_id. LOOK samples every
        # Nth frame and keeps the WHOLE list so it can choose between two tubbies.
        # WAIT/DONE don't look at all.
        look_dets = []
        if state in (CONFIRM, TRACK):
            detection = yolo_detect(frame, frame_w, exclude=visited, want_id=target_id)
            last_detection = detection
        elif state == LOOK and frame_count % YOLO_EVERY_N == 0:
            look_dets = yolo_detect_all(frame, frame_w, exclude=visited)
            last_look_dets = look_dets
            detection = look_dets[0] if look_dets else None   # best, for the overlay
        elif state == LOOK:
            look_dets = last_look_dets                        # skipped frame -> reuse
            detection = look_dets[0] if look_dets else None
        else:
            detection = None              # WAIT / DONE -> not looking

        # ── State machine ─────────────────────────────────────────────────────
        if state == DONE:
            if not done_sent:
                send("DONE")              # ESP stops sweeping; we've already stopped looking
                done_sent = True

        # ── WAIT: idle while the ESP drives. Wait for it to open the window. ──
        elif state == WAIT:
            if not start_sent:
                # TODO: if the ESP self-starts its routine at power-on, DELETE this.
                # Otherwise send whatever kick-off your teammate expects (once).
                # send("START")
                start_sent = True
            if consume("esp_look_start"):        # a look-window opened -> start looking
                state = LOOK
            elif consume("esp_routine_done"):    # ESP ran out of sweeps, still < 2 found
                state = DONE

        # ── LOOK: ESP holds the window open; we watch each sampled frame. ─────
        # NOTE: unlike the reactive file we do NOT send STOP here — the robot keeps
        # moving straight into CONFIRM/TRACK. That's the whole point of the servo.
        elif state == LOOK:
            target, other_side = pick_target(look_dets)   # closest-to-center first
            if target is not None:
                target_id = target[0]
                pending_other_side = other_side           # which way the 2nd tubby sits
                confirm_votes = []
                state = CONFIRM                            # no STOP — stay in motion
            elif consume("esp_look_end"):                  # window closed, nothing found
                state = WAIT

        # ── CONFIRM: vote across frames (on the move) to reject false positives. ─
        elif state == CONFIRM:
            confirm_votes.append(detection[0] if detection is not None else None)
            if len(confirm_votes) >= CONFIRM_FRAMES:
                tally = Counter(v for v in confirm_votes if v is not None)
                winner, count = tally.most_common(1)[0] if tally else (None, 0)
                confirm_votes = []
                if count >= CONFIRM_VOTES:
                    target_id = winner
                    controller.reset()        # fresh controller for this target
                    track_misses = 0
                    centered_streak = 0
                    state = TRACK
                else:
                    target_id = None
                    send("RESUME")
                    state = WAIT

        # ── TRACK: continuous servo. Stream STEER until centered+settled, then STOP. ─
        elif state == TRACK:
            if detection is None:
                track_misses += 1
                centered_streak = 0            # lost sight -> not settled
                if track_misses <= MISS_GRACE:
                    # Brief dropout (flicker / blur). COAST: keep steering the way we
                    # were, decayed, instead of a jerky hard stop. Don't reset the
                    # controller — the target is probably still there.
                    last_steer *= COAST_DECAY
                    send(f"STEER:{last_steer:.3f}")
                elif track_misses <= ALIGN_MAX_MISSES:
                    last_steer = 0.0
                    send("STEER:0.000")        # probably gone -> stop steering, keep looking
                else:
                    # Truly lost. Hand back to the ESP to re-find it; NOT added to
                    # `visited` (we never flashed it), so a re-sweep can pick it up again.
                    target_id = None
                    controller.reset()
                    last_steer = 0.0
                    track_misses = 0
                    send("RESUME")             # plain resume — no turn-back hint here
                    state = WAIT
            else:
                track_misses = 0
                error = detection[2]

                if abs(error) < ALIGN_THRESHOLD:
                    # Centered THIS frame. Stop driving and insist it STAYS centered for
                    # SETTLE_FRAMES frames = "robot has actually halted on the target".
                    send("STOP")
                    last_steer = 0.0
                    send("STEER:0.000")        # zero the correction so STOP isn't fighting it
                    centered_streak += 1
                    if centered_streak >= SETTLE_FRAMES:
                        flash_sent = 0
                        centered_streak = 0
                        state = FLASH
                else:
                    # Off-center: keep DRIVING and stream a continuous correction.
                    centered_streak = 0
                    last_steer = controller.update(error, time.monotonic())
                    send(f"STEER:{last_steer:.3f}")

        # ── FLASH: stopped and settled on the tubby. Pi flashes itself, then resumes. ─
        elif state == FLASH:
            if flash_sent < FLASH_COUNT:
                flash_once()                  # Pi-side flash — NOT a UART command
                flash_sent += 1
            else:
                if target_id is not None:
                    confirmed_ids.add(target_id)
                    visited.add(target_id)    # exclude from all future selection

                if len(confirmed_ids) >= TARGETS_TO_FIND:
                    state = DONE
                else:
                    # No turn to undo (we arced here while driving). Hand the routine
                    # back with a direction hint toward where the OTHER tubby was, so
                    # the ESP turns the right way first instead of sweeping blind.
                    # visited now hides this tubby so the next CONFIRM can only pick B.
                    if pending_other_side is not None:
                        send("RESUME:R" if pending_other_side > 0 else "RESUME:L")
                    else:
                        send("RESUME")
                    state = WAIT

                target_id = None
                pending_other_side = None
                controller.reset()
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
# (Unchanged from the other detectors.)

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
        <title>Detector — continuous servo</title>
        <style>
            body { font-family: monospace; background: #1a1a1a; color: #eee; padding: 20px; }
            img  { display: block; width: 640px; border: 1px solid #444; }
        </style>
    </head>
    <body>
        <h1>Teletubby Detector — CONTINUOUS SERVO (drive + stream STEER until centered)</h1>
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