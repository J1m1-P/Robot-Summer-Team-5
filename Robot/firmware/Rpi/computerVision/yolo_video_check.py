"""
YOLO-on-video check + FPS benchmark  (browser-streaming version)
================================================================================
A throwaway rig to answer two questions BEFORE refactoring to continuous YOLO:

  1. Does YOLO reliably find the tubby in this footage? (The HSV gate never
     triggered on it, so YOLO has literally never looked at the real target yet.)
  2. How fast is inference? Meaningless on your laptop; copy to the Pi, set
     STREAM = False, and it becomes the FPS benchmark that decides every-frame
     vs sample-every-N vs stop-and-scan.

STREAM = True  -> open http://localhost:5000 (or http://<pi-ip>:5000 over wifi)
                  to watch YOLO's boxes on the clip. The clip loops so you can
                  keep watching; the running detection-% and FPS are drawn on it.
STREAM = False -> no browser, no drawing: just rip through the clip once and
                  print the two numbers. This is the clean Pi benchmark.

Unlike the detector, this is a plain viewer with no autonomous loop to keep
alive, so it stays single-threaded — Flask pulling frames IS the loop. (That's
fine here precisely because there's no state machine that must run without a
browser.)
"""

import time
import cv2
from ultralytics import YOLO
from flask import Flask, Response, render_template_string


# ── Knobs ─────────────────────────────────────────────────────────────────────
MODEL_PATH = r"E:\runs\detect\train-9\weights\best_ncnn_model"  # ncnn folder (or best.pt)
VIDEO      = r"E:\ENPH253\TeletubbyImages\TeletubbyImages\tb2.MOV"   # the SAME footage the HSV gate failed on
IMGSZ      = 320          # match your export size
CONF       = 0.5          # match CONFIRM_CONF. Lower (e.g. 0.25) to also see weak
                          # detections — good for judging how confidently YOLO
                          # sees the tubby, not just whether it clears 0.5.
STREAM     = False         # True: watch in a browser. False: FPS-only (the Pi).


model = YOLO(MODEL_PATH)
app = Flask(__name__)


def open_video():
    cap = cv2.VideoCapture(VIDEO)
    if not cap.isOpened():
        raise RuntimeError(f"could not open video: {VIDEO}")
    return cap


def run_yolo(frame):
    """Run one inference, return (annotated_frame, inference_ms, num_boxes)."""
    t0 = time.perf_counter()
    results = model(frame, conf=CONF, imgsz=IMGSZ, verbose=False)
    infer_ms = (time.perf_counter() - t0) * 1000
    # results[0].plot() draws every box + class label + confidence for you.
    annotated = results[0].plot()
    return annotated, infer_ms, len(results[0].boxes)


# ══════════════════════════════════════════════════════════════════════════════
# STREAM MODE — serve the annotated clip to a browser
# ══════════════════════════════════════════════════════════════════════════════
def generate_frames():
    cap = open_video()
    frames = 0            # total frames processed
    total_ms = 0.0        # summed inference time (for the running average)
    hits = 0              # frames where YOLO found >= 1 box

    while True:
        ok, frame = cap.read()
        if not ok:                                  # end of clip -> rewind and loop
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        annotated, infer_ms, n_boxes = run_yolo(frame)
        frames += 1
        total_ms += infer_ms
        if n_boxes > 0:
            hits += 1

        # Draw the two answers right on the feed so you can read them live.
        fps_avg = 1000.0 / (total_ms / frames) if total_ms else 0
        cv2.putText(annotated, f"{infer_ms:5.0f} ms  |  {fps_avg:4.1f} fps avg",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.putText(annotated, f"tubby seen: {hits}/{frames} ({100*hits/frames:.0f}%)",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        ok, buffer = cv2.imencode('.jpg', annotated)
        if ok:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')


@app.route('/')
def index():
    return render_template_string('''
    <html><head><title>YOLO check</title>
    <style>
        body { font-family: monospace; background: #1a1a1a; color: #eee; padding: 20px; }
        img  { display: block; width: 720px; border: 1px solid #444; }
    </style></head>
    <body>
        <h1>YOLO-on-video check — continuous detection</h1>
        <img src="/stream">
    </body></html>
    ''')


@app.route('/stream')
def stream():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


# ══════════════════════════════════════════════════════════════════════════════
# BENCHMARK MODE — no browser, no drawing: rip through once and print the numbers
# ══════════════════════════════════════════════════════════════════════════════
def benchmark():
    cap = open_video()
    frames = 0
    total_ms = 0.0
    hits = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break                                   # one pass, then stop
        # Time inference only; skip .plot() so drawing doesn't skew the numbers.
        t0 = time.perf_counter()
        results = model(frame, conf=CONF, imgsz=IMGSZ, verbose=False)
        total_ms += (time.perf_counter() - t0) * 1000
        frames += 1
        if len(results[0].boxes) > 0:
            hits += 1
    cap.release()

    if frames:
        avg_ms = total_ms / frames
        print(f"\nframes processed : {frames}")
        print(f"tubby seen in    : {hits}/{frames} frames "
              f"({100 * hits / frames:.0f}%)   <- detection reliability")
        print(f"avg inference    : {avg_ms:.0f} ms/frame  ({1000.0 / avg_ms:.1f} FPS)"
              f"   <- sizes continuous-YOLO on the Pi")
    else:
        print("no frames read — check the VIDEO path.")


if __name__ == '__main__':
    if STREAM:
        print("streaming — open http://localhost:5000 to watch.")
        app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)
    else:
        benchmark()