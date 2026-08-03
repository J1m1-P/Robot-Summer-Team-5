"""
HSV + shape Teletubby detector (skeleton).

Pipeline per frame:
  BGR -> HSV -> per-colour mask (with saturation gate) -> contours
       -> area filter -> ellipse-fit "oval" confirm -> detection

Fill in the TODOs. The plumbing is here; the tuning + the two real
decisions (hue bounds, what counts as "oval") are left for you.
"""

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

# Saturation / value gate: kills the white background AND the grey rock.
# White  -> very low S.   Rock (grey/brown) -> low-ish S.
# TODO: tune S_MIN so the rock never survives but the tubbies always do.
S_MIN = 100
V_MIN = 60

# Hue is 0-179 in OpenCV. Red wraps around 0/179 so it needs TWO ranges.
# TODO: point a tubby at the camera, print the HSV of its blob, and tighten
#       these bounds. Values below are rough starting guesses only.
COLOR_HUES = {
    "green":  [(35, 85)],
    "yellow": [(20, 35)],
    "purple": [(125, 160)],
    "red":    [(0, 10), (170, 179)],   # two-piece because red wraps
}

MIN_AREA = 800          # ignore blobs smaller than this (pixels). TODO: tune.

# "Oval" acceptance thresholds. TODO: decide + tune these.
#   - fill ratio  = contour area / ellipse area   (how well an ellipse fits)
#   - aspect      = major axis / minor axis        (reject long thin things)
MIN_FILL_RATIO = 0.70
MAX_ASPECT = 3.0


# ---------------------------------------------------------------------------
# Core steps
# ---------------------------------------------------------------------------

def color_mask(hsv, hue_ranges):
    """Return a binary mask for one colour, with the saturation/value gate applied."""
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for (h_lo, h_hi) in hue_ranges:
        lower = np.array([h_lo, S_MIN, V_MIN])
        upper = np.array([h_hi, 255, 255])
        mask |= cv2.inRange(hsv, lower, upper)

    # TODO: clean up salt-and-pepper noise here.
    # Hint: cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel) then MORPH_CLOSE.
    return mask


def is_oval(contour):
    """Return (ok, info) — does this contour look like an oval tubby body?"""
    if len(contour) < 5:          # fitEllipse needs >= 5 points
        return False, None

    ellipse = cv2.fitEllipse(contour)
    (cx, cy), (axis_a, axis_b), angle = ellipse

    # TODO: compute the two metrics and compare against the thresholds.
    #   major, minor = max(axis_a, axis_b), min(axis_a, axis_b)
    #   aspect = major / minor
    #   ellipse_area = np.pi * (major/2) * (minor/2)
    #   fill_ratio = cv2.contourArea(contour) / ellipse_area
    # Return True only if fill_ratio >= MIN_FILL_RATIO and aspect <= MAX_ASPECT.

    ok = False   # TODO: replace with the real test
    return ok, ellipse


def detect(frame):
    """Return a list of detections: (color, (cx, cy), ellipse)."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    detections = []

    for color, hue_ranges in COLOR_HUES.items():
        mask = color_mask(hsv, hue_ranges)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            if cv2.contourArea(c) < MIN_AREA:
                continue
            ok, ellipse = is_oval(c)
            if not ok:
                continue
            (cx, cy), _, _ = ellipse
            detections.append((color, (int(cx), int(cy)), ellipse))

    # TODO (optional): if two colours claim overlapping blobs, keep the
    #       larger / more oval one so you don't double-report a tubby.
    return detections


# ---------------------------------------------------------------------------
# Runner (for bench-testing on your laptop before porting to the Pi)
# ---------------------------------------------------------------------------

def main():
    cap = cv2.VideoCapture(0)     # TODO: swap for your Pi camera source
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        for color, (cx, cy), ellipse in detect(frame):
            cv2.ellipse(frame, ellipse, (0, 255, 255), 2)
            cv2.putText(frame, color, (cx - 20, cy),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        cv2.imshow("hsv+shape", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()