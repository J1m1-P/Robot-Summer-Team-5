"""
Teletubby Detector — Stage 1 + Stage 2 (done), Stage 4 ML confirm (in progress)
================================================================================
Stage 1 (done): given an HSV mask, find the blob, draw a bounding box,
compute normalized X error. See find_blob() / get_bbox_and_error().

Stage 2 (done): four Teletubby colours + a SEARCH -> ALIGN -> FLASH -> DONE
state machine over them. No UART yet — send() just prints. `po` is still
commented out of COLOR_RANGES pending a retune, and dipsy/laa_laa's ranges
still overlap and need retuning too (see the note above COLOR_RANGES).

Stage 3 (auto-calibration) was designed then deliberately dropped: with
real venue practice access before competition, and no guarantee any
Teletubby is even in frame at startup (whatever's on the field is whatever
you get), a runtime calibration step added startup delay and a new failure
mode (locking onto a stray colour) for little benefit over just tuning
COLOR_RANGES well during venue practice. If lighting drift turns out to be
a real problem later, a single global brightness/exposure compensation
(shift every colour's V-bounds by the difference from practice-session
brightness — doesn't require any Teletubby in view) would be a much
cheaper fix than per-colour hue recalibration.

Stage 4 (ML confirm — in progress): once HSV fires a candidate, an extra
CONFIRM state runs a trained YOLO model on the same frame and cross-checks
YOLO's own bounding box against HSV's. Only if the two boxes agree (enough
overlap) do we commit to ALIGN; otherwise the HSV hit is treated as a false
positive. YOLO runs ONLY in CONFIRM, never every frame, so its per-frame
cost is paid only after HSV has already stopped the robot.

Concepts you need:

  cv2.findContours(mask, mode, method) -> contours, hierarchy
      Traces the outline of every white blob in a binary mask.
      mode=cv2.RETR_EXTERNAL     only outermost contours (ignore holes)
      method=cv2.CHAIN_APPROX_SIMPLE   compress straight-line points

  cv2.contourArea(contour) -> float
      Pixel area enclosed by one contour. Speckle noise produces tiny
      contours — filtering by area is how you separate "real blob"
      from "stray pixel cluster".

  cv2.boundingRect(contour) -> (x, y, w, h)
      Smallest upright rectangle containing the contour. (x, y) is the
      TOP-LEFT corner, not the center.

  Normalized X error:
      box_center_x = x + w / 2
      frame_center_x = frame_width / 2
      error = (box_center_x - frame_center_x) / frame_center_x
      Range: -1.0 (box centered at the left edge) ... 0.0 (centered)
      ... +1.0 (box centered at the right edge).

  YOLO confirm + IoU cross-reference:
      HSV is fast but fires on any background patch that merely shares a
      colour. A trained detector keys on shape/texture instead, so running
      it as a second opinion filters those out. It's a real detector, not a
      crop classifier: it returns its OWN box, and we measure how much that
      box overlaps HSV's using IoU — intersection area / union area, where
      0.0 is no overlap and 1.0 is identical. Enough overlap means both
      detectors agree it's the same object, so the hit is confirmed.
"""

import cv2
import numpy as np
from flask import Flask, Response, render_template_string

cap = cv2.VideoCapture(1)  # change to 0 if you only have one camera
app = Flask(__name__)

# Pixel-area cutoff below which a blob is treated as noise, not the target.
# Depends on your resolution and how close the Teletubby is — start here
# and adjust once you can see real vs. speckle contours.
MIN_CONTOUR_AREA = 500


# ─────────────────────────────────────────────
# STAGE 1 — contour / bbox / X-error (done)
# ─────────────────────────────────────────────

def find_blob(mask, min_area=MIN_CONTOUR_AREA):
    """
    Find the largest contour in `mask` above `min_area`.
    Returns a single contour, or None if nothing qualifies.
    """
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    largest = max(contours, key=cv2.contourArea)

    if cv2.contourArea(largest) < min_area:
        return None

    return largest


def get_bbox_and_error(contour, frame_width):
    """
    Given a contour and the frame width in pixels, return
    (x, y, w, h, error) where error is the normalized X error, [-1, +1].
    """
    x, y, w, h = cv2.boundingRect(contour)

    box_center_x = x + w / 2
    frame_center_x = frame_width / 2
    error = (box_center_x - frame_center_x) / frame_center_x

    return x, y, w, h, error


# ─────────────────────────────────────────────
# STAGE 2 — four colours + state machine (done, no UART yet)
# ─────────────────────────────────────────────
# Tune each colour with hsv_tuner.py, ideally at the actual competition
# venue under real lighting. Hold up EVERY other colour while tuning one,
# and adjust until only the intended colour lights up in the mask —
# that's how you avoid range overlap between colours.
#
# KNOWN ISSUE (not yet fixed): dipsy and laa_laa's hue ranges still overlap
# (dipsy 23-73, laa_laa 15-57 -> overlap 23-57). `po` is commented out
# entirely until it's tuned — its old wide placeholder matched nearly the
# whole frame and caused false "second Teletubby" hits.

COLOR_RANGES = {
    "tinky_winky": [(np.array([127, 24, 23]), np.array([154, 174, 199]))],
    "dipsy":       [(np.array([23, 26, 15]), np.array([73, 195, 207]))],
    "laa_laa":     [(np.array([15, 26, 15]), np.array([57, 195, 207]))],
    # "po":        [(np.array([0, 0, 0]), np.array([0, 0, 0])),
    #               (np.array([0, 0, 0]), np.array([0, 0, 0]))],
}

ALIGN_THRESHOLD   = 0.08   # |error| below this counts as "centred"
FLASH_COUNT       = 3      # how many times to "flash" once centred
TARGETS_TO_FIND   = 2      # how many Teletubbies to find per run (not all 4)


def build_mask(hsv_frame, ranges):
    """
    OR together the mask from every (lower, upper) pair in `ranges`.
    Most colours have one pair; Po has two (hue wraps at 0/179).
    """
    combined = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
    for lower, upper in ranges:
        combined = cv2.bitwise_or(combined, cv2.inRange(hsv_frame, lower, upper))
    return combined


def scan_all(hsv_frame, frame_width, exclude=None):
    """
    Check every colour in COLOR_RANGES except those in `exclude`.
    Returns a dict: name -> (x, y, w, h, error) for each colour currently
    detected in-frame.
    """
    exclude = exclude or set()
    found = {}

    for name, ranges in COLOR_RANGES.items():
        if name in exclude:
            continue
        mask = build_mask(hsv_frame, ranges)
        contour = find_blob(mask)
        if contour is not None:
            found[name] = get_bbox_and_error(contour, frame_width)

    return found


def send(message):
    """
    Stand-in for the eventual UART link to the ESP32. Prints what WOULD be
    sent for now — swap the body for serial.write() once hardware is wired up.
    """
    print(f"[TX] {message}")


# ─────────────────────────────────────────────
# STAGE 4 — YOLO confirm step (in progress)
# ─────────────────────────────────────────────
# After HSV fires and the robot stops, this step runs a trained YOLO model on
# the frame to independently confirm a real Teletubby is there, then compares
# YOLO's box against HSV's box with IoU. Enough overlap = both detectors agree
# = we trust the hit and move to ALIGN. Runs only in CONFIRM, never per-frame.

from ultralytics import YOLO   # heavy import (pulls in torch) — dev machine only for now

MODEL_PATH   = "..."   # TODO: path to your trained .pt (e.g. "runs/detect/train/weights/best.pt")
CONFIRM_CONF = 0.5     # TODO: tune — min YOLO confidence for a detection to count
CONFIRM_IOU  = 0.3     # TODO: tune — min overlap with the HSV box to call it the same object

# Load the weights ONCE here, not inside the loop — re-reading them every
# frame would be enormously slow.
model = YOLO(MODEL_PATH)   # TODO: set MODEL_PATH above before running this file


def iou(box_a, box_b):
    """
    Intersection-over-Union of two boxes in (x1, y1, x2, y2) corner form.
    0.0 = no overlap, 1.0 = identical. This is the number we threshold on to
    decide whether HSV's box and YOLO's box describe the same object.
    """
    # TODO:
    #   inter_x1 = max of the two left edges;   inter_y1 = max of the two tops
    #   inter_x2 = min of the two right edges;  inter_y2 = min of the two bottoms
    #   inter_w  = max(0, inter_x2 - inter_x1); inter_h = max(0, inter_y2 - inter_y1)
    #   intersection = inter_w * inter_h
    #   area_a, area_b = each box's width * height
    #   union = area_a + area_b - intersection
    #   return intersection / union   (guard union == 0 -> return 0.0)
    ...

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
    Run YOLO on the full BGR `frame` and decide whether it agrees with HSV.
    `hsv_bbox` is (x, y, w, h) from get_bbox_and_error / boundingRect. Returns
    True if some YOLO detection overlaps HSV's box by at least min_iou.

    NOTE: ultralytics takes BGR numpy arrays directly, so pass `frame` (the
    raw camera image), NOT the hsv-converted one.
    """
    # Convert HSV's (x, y, w, h) top-left+size box into (x1, y1, x2, y2) corners
    # so it's in the same format as YOLO's boxes for iou().
    # TODO: hsv_corners = (x, y, x + w, y + h)
    x, y, w, h = hsv_bbox
    hsv_corners = (x, y, x + w, y + h)
    results = model(frame, conf=conf, verbose=False)   # returns a list; use results[0]
    yolo_boxes = results[0].boxes.xyxy.cpu().numpy()  # (N, 4) corner boxes
    yolo_confs = results[0].boxes.conf.cpu().numpy()   # (N,) confidences
    #   results[0].boxes.xyxy -> tensor (N, 4) corner boxes
    #   results[0].boxes.conf -> tensor (N,)   confidences
    #   call .cpu().numpy() on a tensor before using it as plain numbers

    # TODO:
    #   if there are no boxes -> return False
    #   for each yolo box: compute iou(yolo_box, hsv_corners)
    #   return True if the best iou >= min_iou, else False
    if len(yolo_boxes) == 0:
        return False
    else:
        best_iou = max(iou(yolo_box, hsv_corners) for yolo_box in yolo_boxes)
        return best_iou >= min_iou


SEARCH, CONFIRM, ALIGN, FLASH, DONE = "SEARCH", "CONFIRM", "ALIGN", "FLASH", "DONE"

# Mutable control-loop state. generate_frames() is the only place reading
# the camera, so it's also the only place that advances the state machine.
state       = SEARCH
visited     = set()
target_name = None
flash_sent  = 0
done_sent   = False   # so the DONE branch only fires send("DONE") once


# ─────────────────────────────────────────────
# GIVEN — drawing + Flask plumbing
# ─────────────────────────────────────────────

def draw_overlay(frame, bbox_error, state, target_name):
    """Draws bounding box, center dot, frame-center line, error, and state."""
    cv2.putText(frame, f"state: {state}  target: {target_name}", (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    if bbox_error is None:
        cv2.putText(frame, "no blob", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        return frame

    x, y, w, h, error = bbox_error
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.circle(frame, (x + w // 2, y + h // 2), 4, (0, 0, 255), -1)
    cx = frame.shape[1] // 2
    cv2.line(frame, (cx, 0), (cx, frame.shape[0]), (255, 255, 0), 1)
    cv2.putText(frame, f"err: {error:+.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    return frame


def generate_frames():
    global state, visited, target_name, flash_sent, done_sent

    while True:
        success, frame = cap.read()
        if not success:
            break

        frame_w = frame.shape[1]
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        current_bbox_error = None

        if state == DONE:
            if not done_sent:
                send("DONE")
                done_sent = True

        elif state == SEARCH:
            detections = scan_all(hsv, frame_w, exclude=visited)
            if detections:
                print("SEARCH saw:", list(detections.keys()))
                target_name = next(iter(detections))
                send("STOP")
                state = CONFIRM   # was ALIGN — now YOLO gets a say first
            else:
                send("TURN:0.3")  # placeholder constant scan speed

        elif state == CONFIRM:
            # Robot is already stopped. Re-detect the target colour on THIS
            # frame to get a fresh HSV box, then ask YOLO for a second opinion
            # on the same frame so the two boxes are directly comparable.
            mask = build_mask(hsv, COLOR_RANGES[target_name])
            contour = find_blob(mask)
            if contour is None:
                # HSV lost the candidate while stopping — nothing to confirm.
                send("STOP")
                target_name = None
                state = SEARCH
            else:
                current_bbox_error = get_bbox_and_error(contour, frame_w)
                x, y, w, h, _ = current_bbox_error
                if yolo_confirm(frame, (x, y, w, h)):
                    # Both detectors agree -> trust it, start centring.
                    state = ALIGN
                else:
                    # TODO: false positive. Dropping straight back to SEARCH will
                    # re-fire on this same patch next frame (SEARCH<->CONFIRM
                    # ping-pong). Decide how to break out — e.g. send a small TURN
                    # to sweep the patch out of view before resuming SEARCH.
                    send("TURN")  # sweep the patch out of view
                    target_name = None
                    state = SEARCH

        elif state == ALIGN:
            mask = build_mask(hsv, COLOR_RANGES[target_name])
            contour = find_blob(mask)
            if contour is None:
                send("STOP")
                state = SEARCH
                target_name = None
            else:
                current_bbox_error = get_bbox_and_error(contour, frame_w)
                _, _, _, _, error = current_bbox_error
                if abs(error) < ALIGN_THRESHOLD:
                    send("STOP")
                    state = FLASH
                    flash_sent = 0
                else:
                    send(f"TURN:{error:.3f}")

        elif state == FLASH:
            if flash_sent < FLASH_COUNT:
                send("FLASH")
                flash_sent += 1
            else:
                visited.add(target_name)
                target_name = None
                flash_sent = 0
                state = DONE if len(visited) >= TARGETS_TO_FIND else SEARCH

        display = draw_overlay(frame.copy(), current_bbox_error, state, target_name)

        _, buffer = cv2.imencode('.jpg', display)
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


@app.route('/')
def index():
    html = '''
    <html>
    <head>
        <title>Detector</title>
        <style>
            body { font-family: monospace; background: #1a1a1a; color: #eee; padding: 20px; }
            img  { display: block; width: 640px; border: 1px solid #444; }
        </style>
    </head>
    <body>
        <h1>Teletubby Detector — 4 colours + state machine</h1>
        <img src="/stream">
    </body>
    </html>
    '''
    return render_template_string(html)


@app.route('/stream')
def stream():
    return Response(generate_frames(),
                     mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)