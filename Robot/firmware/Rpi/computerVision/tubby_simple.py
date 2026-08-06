"""
Teletubby Detector — SIMPLE FLASH MODE.

FLOW
  IDLE      Camera runs, no YOLO; state/detections print to console.
  SCANNING  Vote presence across SCAN_BURST_FRAMES (SCAN_MIN_VOTES needed).
            Identity is ignored -- every detection is the same "teletubby"
            class. A solid detection flashes immediately and counts toward
            TARGETS_TO_FIND. One report per request, then back to IDLE.

WIRE (robot_common/pi_action_packet.h) -- identical to tubby_detector.py.
  ESP->Pi PI_REQUEST: request_id, action, parameter (unused).
  Pi->ESP PI_REPORT:  request_id, result, target_id, horizontal_error
                      (always 0 here), confidence_percent.
  ALL_FOUND is a flag, not a detection: horizontal_error is ignored.
"""

import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import ncnn
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

# Force line-buffered output so prints show up immediately in the systemd
# journal. A pipe (which is what the journal is) block-buffers stdout by
# default, so without this every [startup] line is invisible until the process
# exits -- which is why the journal looked empty.
sys.stdout.reconfigure(line_buffering=True)
sys.stderr.reconfigure(line_buffering=True)


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
SCAN_BURST_FRAMES   = 5    # ADJUST: frames sampled per request (well under the ESP's 15s timeout)
SCAN_MIN_VOTES      = 3    # ADJUST: burst frames that must agree before flashing
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
FLASH_PIN      = 18        # ADJUST: BCM GPIO number driving the flash
FLASH_ON_TIME  = 0.5        # ADJUST: seconds the flash stays on per pulse
FLASH_OFF_TIME = 0.15      # ADJUST: gap between pulses in a multi-flash sequence

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
# YOLO DETECTION (direct NCNN — no ultralytics / torch)
# ══════════════════════════════════════════════════════════════════════════════
# We load the NCNN export directly with the `ncnn` runtime instead of going
# through ultralytics' YOLO() wrapper. That skips importing torch (~30s on a
# Pi), which NCNN inference never actually uses. The tradeoff: we now do the
# letterbox / decode / NMS by hand below -- work ultralytics used to hide.
#
# Requires the ncnn python package on the Pi:   pip install ncnn

NUM_CLASSES = 2        # model has 2 classes (output was (6, 8400) = 4 box + 2 cls)
NMS_IOU     = 0.45     # ADJUST: IoU threshold for non-max suppression

# ultralytics names the export files model.ncnn.param / model.ncnn.bin inside
# the *_ncnn_model directory. VERIFY these match what's in your folder.
PARAM_PATH = os.path.join(MODEL_PATH, "model.ncnn.param")
BIN_PATH   = os.path.join(MODEL_PATH, "model.ncnn.bin")

print(f"[startup] loading NCNN model from {MODEL_PATH} ...")
for _p in (PARAM_PATH, BIN_PATH):
    if not os.path.exists(_p):
        print(f"[startup] FAILED: missing {_p}")
        raise FileNotFoundError(_p)

net = ncnn.Net()
net.opt.num_threads = 4        # ADJUST: the Pi has 4 cores; use them all
net.load_param(PARAM_PATH)
net.load_model(BIN_PATH)
print("[startup] NCNN model loaded")

# ultralytics exports name the blobs "in0" (input) and "out0" (output). VERIFY
# by opening model.ncnn.param: the first layer's output name is the input blob,
# the last layer's output name is the output blob.
INPUT_BLOB  = "in0"    # ADJUST if your .param uses a different name
OUTPUT_BLOB = "out0"   # ADJUST if your .param uses a different name

# Print the raw output shape once, on the first inference, so you can confirm
# the decode orientation below is right.
_debug_printed_shape = False


def _letterbox(frame):
    """Resize keeping aspect ratio and pad to IMGSZ x IMGSZ with gray (114),
    matching ultralytics' preprocessing. Returns (canvas, scale, pad_left,
    pad_top) so boxes can be mapped back to original-frame coordinates."""
    h, w = frame.shape[:2]
    scale = min(IMGSZ / h, IMGSZ / w)
    nh, nw = int(round(h * scale)), int(round(w * scale))
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_top  = (IMGSZ - nh) // 2
    pad_left = (IMGSZ - nw) // 2
    canvas = np.full((IMGSZ, IMGSZ, 3), 114, dtype=np.uint8)
    canvas[pad_top:pad_top + nh, pad_left:pad_left + nw] = resized
    return canvas, scale, pad_left, pad_top


def yolo_detect_all(frame, frame_w):
    """
    Run the NCNN model on one frame; return every detection as
    ((x, y, w, h), error), highest-confidence first, error in
    -1 (left) .. +1 (right). Drop-in replacement for the old ultralytics
    version -- identical return shape, so nothing downstream changes.
    """
    global _debug_printed_shape

    # 1. Preprocess: letterbox, then hand the pixels to ncnn as RGB, /255.
    canvas, scale, pad_left, pad_top = _letterbox(frame)
    mat_in = ncnn.Mat.from_pixels(
        canvas, ncnn.Mat.PixelType.PIXEL_BGR2RGB, IMGSZ, IMGSZ)
    mat_in.substract_mean_normalize([0.0, 0.0, 0.0],
                                    [1 / 255.0, 1 / 255.0, 1 / 255.0])

    # 2. Inference.
    ex = net.create_extractor()
    ex.input(INPUT_BLOB, mat_in)
    _ret, mat_out = ex.extract(OUTPUT_BLOB)
    out = np.squeeze(np.array(mat_out))

    # 3. Normalise to shape (4 + NUM_CLASSES, num_anchors).
    if not _debug_printed_shape:
        print(f"[detect] raw NCNN output shape = {out.shape}  "
              f"(expect ({4 + NUM_CLASSES}, N))")
        _debug_printed_shape = True
    if out.shape[0] != 4 + NUM_CLASSES:
        out = out.T                      # handle (num_anchors, 4+NC) orientation

    # 4. Decode. Rows 0-3 = cx, cy, w, h in letterbox pixels; rows 4+ = class
    #    scores (already 0..1 from the export). Keep the max class score.
    cx, cy, bw, bh = out[0], out[1], out[2], out[3]
    scores = out[4:].max(axis=0)

    keep = scores >= DETECT_CONF
    if not np.any(keep):
        return []
    cx, cy, bw, bh, scores = cx[keep], cy[keep], bw[keep], bh[keep], scores[keep]

    # Convert center-xywh (letterbox space) -> top-left xywh (original frame).
    xs = (cx - bw / 2 - pad_left) / scale
    ys = (cy - bh / 2 - pad_top) / scale
    ws = bw / scale
    hs = bh / scale

    # 5. NMS via OpenCV (wants [x, y, w, h] int boxes).
    boxes_xywh = [[int(xs[i]), int(ys[i]), int(ws[i]), int(hs[i])]
                  for i in range(len(scores))]
    idxs = cv2.dnn.NMSBoxes(boxes_xywh, scores.tolist(), DETECT_CONF, NMS_IOU)
    if len(idxs) == 0:
        return []
    idxs = np.array(idxs).flatten()

    # 6. Build ((x, y, w, h), error) tuples, highest confidence first.
    order = idxs[np.argsort(scores[idxs])[::-1]]
    dets = []
    for i in order:
        bx, by, bwi, bhi = boxes_xywh[i]
        error = ((bx + bwi / 2) - frame_w / 2) / (frame_w / 2)   # -1 left .. +1 right
        dets.append(((bx, by, bwi, bhi), error))
    return dets


# ── warm up ───────────────────────────────────────────────────────────────────
# ncnn loads the weights at load_model() above, but the first extract still
# allocates layer buffers. Force that now on a dummy frame so the readiness
# beacon and the first real scan aren't paying that cost.
print("[startup] warming up model ...")
_warmup_start = time.time()
yolo_detect_all(np.zeros((CAMERA_HEIGHT, CAMERA_WIDTH, 3), dtype=np.uint8),
                CAMERA_WIDTH)
print(f"[startup] warmup done in {time.time() - _warmup_start:.2f}s")

# Count of teletubbies flashed so far (identity is never tracked).
found_count = 0


def pick_target(detections):
    """Closest-to-center detection, or None if the frame has nothing."""
    if not detections:
        return None
    return min(detections, key=lambda d: abs(d[1]))


# ══════════════════════════════════════════════════════════════════════════════
# ANSWERING ONE SCAN REQUEST
# ══════════════════════════════════════════════════════════════════════════════

# Flash timing is a small non-blocking state machine serviced once per
# control_loop tick instead of GPIO-HIGH -> time.sleep(FLASH_ON_TIME) ->
# GPIO-LOW -- that blocking version stalled the whole UART/scan loop for
# FLASH_ON_TIME seconds per pulse.
_flash_pulses_pending = 0
_flash_state    = "idle"      # "idle" | "on" | "gap"
_flash_deadline = 0.0


def flash_once():
    """Queue one flash pulse via GPIO18. Returns immediately -- the actual
    GPIO timing happens in _service_flash() on later control_loop ticks."""
    global _flash_pulses_pending
    _flash_pulses_pending += 1


def _service_flash():
    """Advance the flash state machine by however much time has passed.
    Must be polled every control_loop tick; a no-op when nothing's queued."""
    global _flash_state, _flash_deadline, _flash_pulses_pending
    now = time.time()

    if _flash_state == "idle":
        if _flash_pulses_pending == 0:
            return
        _flash_pulses_pending -= 1
        _flash_state = "on"
        _flash_deadline = now + FLASH_ON_TIME
        if GPIO is None:
            print("[FLASH] (Pi-side, no GPIO — dev machine)")
        else:
            GPIO.output(FLASH_PIN, GPIO.HIGH)
            print("[FLASH] (Pi-side)")
        return

    if now < _flash_deadline:
        return

    if _flash_state == "on":
        if GPIO is not None:
            GPIO.output(FLASH_PIN, GPIO.LOW)
        _flash_state = "gap"
        _flash_deadline = now + FLASH_OFF_TIME
    else:  # "gap"
        _flash_state = "idle"


def _shutdown(code):
    """Exit once every target is flashed. SystemExit still runs the
    `finally` blocks in control_loop()/__main__ since we're on the main
    thread."""
    if GPIO is not None:
        GPIO.output(FLASH_PIN, GPIO.LOW)
    sys.exit(code)


def send_report(request_id, result, target_id=0, horizontal_error=0.0,
                 confidence_percent=0):
    """Send one PiReportPacket, or just print it in dev mode (link is None)."""
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
    Vote presence across a short burst of frames and flash immediately if
    solid -- no centering, no identity, one report per call. `parameter` is
    decoded but unused (reserved for future per-scan tuning).
    """
    global found_count

    # Sample the burst, counting frames with at least one detection.
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

    # Camera never produced a usable frame this burst.
    if frames_read == 0:
        print(f"[SCAN #{request_id}] camera unavailable")
        send_report(request_id, PI_RESULT_CAMERA_FAULT)
        return

    # Not enough of the burst agreed -- treat as noise, not a detection.
    if frames_seen < SCAN_MIN_VOTES:
        print(f"[SCAN #{request_id}] no detection (found so far={found_count})")
        send_report(request_id, PI_RESULT_NOT_FOUND)
        return

    confidence_percent = round(100 * frames_seen / SCAN_BURST_FRAMES)

    # Detection locked in: flash and count it. Relies on the caller only
    # requesting one scan per physical teletubby.
    for _ in range(FLASH_COUNT):
        flash_once()
    found_count += 1
    print(f"[SCAN #{request_id}] flashed teletubby #{found_count} conf={confidence_percent}%")

    # Tell the ESP whether that was the last target.
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
        # control_loop won't get another tick to service the queued flash
        # pulses once we shut down, so drain them here. Blocking is fine --
        # we're exiting either way.
        while _flash_state != "idle" or _flash_pulses_pending:
            _service_flash()
            time.sleep(0.03)
        print("[control_loop] all targets found — shutting down")
        _shutdown(0)


# ══════════════════════════════════════════════════════════════════════════════
# CONSOLE STATE REPORTING — replaces the old Flask/browser view
# ══════════════════════════════════════════════════════════════════════════════

_last_reported_state = None


def _report_state(label, detection=None):
    """Print state transitions once each; print a detection only when seen."""
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
# Camera's open and the model's warm, so tell the arm ESP it's safe to start
# relaying scan requests. Repeated at READY_REPEAT_INTERVAL_S -- like
# arm_action_dispatcher.cpp's own startup beacon -- so one lost byte can't
# strand the ESP; it stops once a request proves the ESP already knows.
# Also flash once here (a single pulse, not the two-flash find pattern) so
# readiness is visible on the bench without a laptop attached.
flash_once()

READY_REPEAT_INTERVAL_S = 0.2
_ready_pending = True
_last_ready_sent_time = 0.0


def _reopen_camera():
    """Reopen a dead camera once; True only if the fresh handle reads a frame."""
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
    """Save one raw idle frame to COLLECT_DIR at most every COLLECT_INTERVAL_S.
    No-op unless COLLECT_IMAGES is on."""
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
            # Advance any queued flash pulses -- non-blocking, so this must
            # run every tick regardless of what else happens below.
            _service_flash()

            # UART: nudge the readiness beacon if unacknowledged, then
            # answer any scan requests queued since the last pass.
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

            # One idle-loop camera read. Failures count against the retry
            # budget; MAX_FAILURES in a row triggers a reopen attempt.
            ok, frame = cap.read()
            if not ok:
                consecutive_failures += 1
                if consecutive_failures >= MAX_FAILURES:
                    if _reopen_camera():
                        consecutive_failures = 0
                    else:
                        # No point sitting here silently -- the ESP would
                        # just keep re-requesting scans that time out.
                        print("[control_loop] camera unavailable after reopen attempt — giving up.")
                        _shutdown(1)
                time.sleep(0.03)
                continue
            consecutive_failures = 0

            # Genuinely idle: no scan requested this tick. Optionally bank
            # the frame for training data, log state, loop again.
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