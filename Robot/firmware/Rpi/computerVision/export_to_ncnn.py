"""
Export best.pt -> NCNN for the Raspberry Pi 4
================================================================================
Run this ONCE on your dev machine (where ultralytics + best.pt live). It produces
a `best_ncnn_model/` folder that you copy to the Pi and load with
`YOLO("best_ncnn_model")` — the same wrapper, so yolo_confirm() in detector_final
doesn't change.

WHY NCNN: it's Ultralytics' fastest backend for YOLO on Pi 4's ARM CPU, and it
loads back through the normal YOLO() API, so the inference call stays identical.

Deps (dev machine): `pip install ncnn` — or just run this and let ultralytics
auto-install it on first export. On the Pi you'll also need ncnn present for
inference (see the note at the bottom).
"""

import glob
from pathlib import Path

import numpy as np
from ultralytics import YOLO


# ── Inputs / knobs ────────────────────────────────────────────────────────────
PT_MODEL = r"E:\runs\detect\train-9\weights\best.pt"

# DECISION 1 — input size. This is your biggest FPS lever on the Pi.
#   640: full accuracy, ~2-5 FPS on Pi 4. Start here.
#   320: ~2-4x faster, some accuracy loss. Drop to this ONLY if CONFIRM drags.
# Whatever you pick here, detector_final must run inference at the SAME size
# (pass imgsz=... in yolo_confirm's model() call), or you lose the speed / the
# boxes shift.
IMGSZ = 640   # TODO: confirm 640, or set 320 after benchmarking on the Pi.

# DECISION 2 — fp16. Near-free accuracy, faster. Keep True.
# Do NOT switch to int8 export unless FPS forces it: int8 needs a calibration set
# and can smear the dipsy/laa_laa separation you tuned for.
HALF = True

# The folder export() produces (printed when it runs). It sits next to best.pt.
NCNN_DIR = r"E:\runs\detect\train-9\weights\best_ncnn_model"

# Confidence sanity_check() runs both models at -- matches the 0.5 default
# used elsewhere in this codebase (tubby_detector.py's DETECT_CONF), so the
# comparison reflects real match-time conditions.
SANITY_CHECK_CONF = 0.5

# ── LINK YOUR TEST IMAGES HERE ────────────────────────────────────────────────
# JPEGs that contain tubbies — include at least one with dipsy AND laa_laa in
# frame, since that's the call fp16 is most likely to disturb. Point this at
# a folder of such images (grab frames from your test video, or snap photos).
# These are what the .pt and NCNN models get compared on.
TEST_IMAGES_DIR = r"E:\ENPH253\TeletubbyImages\Labeled3\valid\images"
TEST_IMAGES = glob.glob(str(Path(TEST_IMAGES_DIR) / "*.jpg"))


def export():
    """Load the .pt weights and emit best_ncnn_model/ next to them."""
    model = YOLO(PT_MODEL)
    model.export(format="ncnn", imgsz=IMGSZ, half=HALF)
    # The result is a directory (e.g. E:\runs\...\weights\best_ncnn_model\),
    # NOT a single file — copy the whole folder to the Pi. Ultralytics prints
    # the exact output path when this runs — it should match NCNN_DIR above.


def iou(box_a, box_b):
    """
    Intersection-over-Union of two boxes given as (x1, y1, x2, y2) corners.
    It's the overlap area divided by the combined area: 0.0 = no overlap,
    1.0 = identical. Used to check the pt model's box and the ncnn model's
    box describe the SAME object, not two different detections.
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


def sanity_check(ncnn_dir, image_paths):
    """
    Verify the export didn't change the model's answers. Export can subtly shift
    boxes/scores (layer fusion, fp16 rounding), and a shift that flips a
    dipsy/laa_laa call would quietly break CONFIRM. So compare the two models on a
    handful of representative frames BEFORE trusting the NCNN version.

    Prints a [MISMATCH] line for anything worth eyeballing; a clean run (no
    mismatches) means the NCNN model is safe to swap in.
    """
    import cv2

    if not image_paths:
        print(f"[sanity_check] no test images found under {TEST_IMAGES_DIR} "
              f"-- point TEST_IMAGES_DIR at a real folder of tubby photos first")
        return

    IOU_SAME_OBJECT = 0.8   # below this, the two boxes point at different things

    print(f"[sanity_check] comparing {PT_MODEL} vs {ncnn_dir} "
          f"on {len(image_paths)} image(s)")
    pt   = YOLO(PT_MODEL)   # original fp32 weights — the "ground truth" here
    ncnn = YOLO(ncnn_dir)   # the exported fp16 NCNN model we're vetting
    names = pt.names        # class-index -> short name (e.g. {0:'DP', 1:'LL', ...})

    mismatches = 0
    for path in image_paths:
        frame = cv2.imread(path)
        if frame is None:                       # bad path / unreadable file
            print(f"[SKIP] could not read {path}")
            continue

        # Run both models under identical, real-world settings.
        rp = pt(frame,   conf=SANITY_CHECK_CONF, imgsz=IMGSZ, verbose=False)[0]
        rn = ncnn(frame, conf=SANITY_CHECK_CONF, imgsz=IMGSZ, verbose=False)[0]

        pt_boxes   = rp.boxes.xyxy.cpu().numpy()
        pt_cls     = rp.boxes.cls.cpu().numpy().astype(int)
        ncnn_boxes = rn.boxes.xyxy.cpu().numpy()
        ncnn_cls   = rn.boxes.cls.cpu().numpy().astype(int)

        # Check 1 — same number of detections on this frame?
        if len(pt_boxes) != len(ncnn_boxes):
            print(f"[MISMATCH] {path}: pt found {len(pt_boxes)} box(es), "
                  f"ncnn found {len(ncnn_boxes)}")
            mismatches += 1
            continue

        # Check 2 — match each pt box to its best-overlapping ncnn box, then
        # confirm they're the SAME object and, crucially, the SAME class.
        for i, pbox in enumerate(pt_boxes):
            if len(ncnn_boxes) == 0:
                print(f"[MISMATCH] {path}: pt box {i} has no ncnn box to match")
                mismatches += 1
                break

            ious = [iou(pbox, nbox) for nbox in ncnn_boxes]  # overlap vs every ncnn box
            best = int(np.argmax(ious))                      # index of the closest one

            if ious[best] < IOU_SAME_OBJECT:
                # Boxes don't line up — the models are looking at different spots.
                print(f"[MISMATCH] {path}: pt box {i} ({names[pt_cls[i]]}) "
                      f"has no ncnn box overlapping it (best IoU {ious[best]:.2f})")
                mismatches += 1
            elif pt_cls[i] != ncnn_cls[best]:
                # THE important failure: same object, different identity call.
                print(f"[MISMATCH] {path}: same box, different class — "
                      f"pt says {names[pt_cls[i]]}, ncnn says {names[ncnn_cls[best]]}")
                mismatches += 1

    print(f"\nsanity check done — {mismatches} mismatch(es). "
          f"{'Safe to swap in.' if mismatches == 0 else 'Eyeball the frames above.'}")


if __name__ == "__main__":
    # Step 1: export. Run once, note the printed best_ncnn_model path (should
    # match NCNN_DIR above — fix NCNN_DIR if your weights live elsewhere).
    #export()

    # Step 2: once export has produced the folder and you've filled in
    # TEST_IMAGES, comment out export() above and uncomment this to vet it:
    sanity_check(NCNN_DIR, TEST_IMAGES)


# ── Wiring it into detector_final.py (do this AFTER sanity_check passes) ───────
# The change is tiny, by design:
#   1. Point MODEL_PATH at the NCNN FOLDER instead of the .pt file:
#        MODEL_PATH = r"...\best_ncnn_model"       # a directory, not a file
#      YOLO(MODEL_PATH) auto-detects the NCNN format — no other load change.
#   2. If you chose IMGSZ = 320, pass it through in yolo_confirm's call so
#      inference matches the export size:
#        results = model(frame, conf=conf, imgsz=320, verbose=False)
#   3. Consider keeping BOTH paths behind a flag so the same file still runs on
#      your dev machine (.pt) and the Pi (ncnn) — e.g. pick MODEL_PATH off the
#      same ENABLE_STREAM-style toggle, or a new USE_NCNN flag.
#
# ── On the Pi ─────────────────────────────────────────────────────────────────
# Install the runtime: `pip install ncnn` (plus ultralytics). Copy the whole
# best_ncnn_model/ folder over. Then benchmark: time a CONFIRM cycle (5 frames)
# and decide whether 640 is fast enough or you drop to 320.