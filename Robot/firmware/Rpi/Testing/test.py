"""
Headless test harness for detector_final.py
================================================================================
Runs the REAL control loop and state machine, but with no robot and no ESP:
  * the camera is pointed at a VIDEO FILE (or a live cam) instead of the course,
  * SERIAL_PORT stays None, so send() just prints "[TX] ..." to the console,
  * the ESP's "scan sweep finished" signal is FAKED on a timer.

The point is to watch the state machine advance ON ITS OWN — SCAN -> CONFIRM ->
ALIGN -> FLASH -> FOLLOW — driven by the footage, not by a browser pulling frames.
That's exactly the behaviour the threading refactor changed, so this is where you
shake out threading bugs (races on confirmed_ids, the loop wedging, frames not
publishing) cheaply at your desk, before the Pi or the ESP exist.

This file imports detector_final and OVERRIDES a few of its globals before
starting the loop. It never edits detector_final itself.
"""

import time
import threading
import cv2

import teletubby_detector as det   # importing runs its config: opens cam 1, loads best.pt


# ── What to feed the loop ─────────────────────────────────────────────────────
# A path to a video clip of tubbies moving through frame gives you repeatable
# runs (same footage every time). Use an integer instead (0 or 1) to test against
# a live webcam through the harness.
VIDEO_SOURCE = r"E:\ENPH253\TeletubbyImages\TeletubbyImages\tb9.MOV"

# Seconds before we pretend the ESP finished its scan sweep. Only matters if no
# tubby is visible during SCAN: it lets the machine fall through SCAN -> FOLLOW
# the way the real ESP signal would. If a tubby IS visible, SCAN -> CONFIRM fires
# first and this never comes into play.
FAKE_SCAN_AFTER = 5.0


# ── A camera stand-in that loops the clip forever ─────────────────────────────
class LoopingCapture:
    """
    Duck-types the slice of cv2.VideoCapture that control_loop uses (.read() and
    .release()), but rewinds to frame 0 at end-of-file so a short clip plays on a
    loop. Without this, the clip would end, cap.read() would return False, and
    control_loop's failure counter would (correctly) stop the loop after ~1 s.
    """
    def __init__(self, source):
        self._cap = cv2.VideoCapture(source)
        if not self._cap.isOpened():
            raise RuntimeError(f"could not open video source: {source!r}")

    def read(self):
        ok, frame = self._cap.read()
        if not ok:                                    # hit the end of the clip...
            self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # ...rewind...
            ok, frame = self._cap.read()               # ...and read the first frame
        return ok, frame

    def release(self):
        self._cap.release()


def fake_esp_scan_finished(after):
    """After `after` seconds, flip detector_final's ESP flag so SCAN can fall
    through to FOLLOW. Stands in for the real 'sweep complete' packet."""
    time.sleep(after)
    det.esp_scan_done = True
    print(f"[harness] faked ESP 'scan finished' after {after:.0f}s")


if __name__ == "__main__":
    # 1. Swap the camera. Import already opened webcam 1 in detector_final — let
    #    it go, then point det.cap at our looping video. control_loop reads the
    #    module global det.cap, so reassigning it here is all that's needed.
    det.cap.release()
    det.cap = LoopingCapture(VIDEO_SOURCE)

    # 2. Keep the browser view on so you can watch, and confirm the loop advances
    #    without a browser: the states will already be moving before you open it.
    det.ENABLE_STREAM = True

    # 3. Fake the ESP sweep-done signal on a timer (daemon: dies with the process).
    threading.Thread(target=fake_esp_scan_finished,
                     args=(FAKE_SCAN_AFTER,), daemon=True).start()

    # 4. Start the real control loop on its own thread, then serve Flask — exactly
    #    how detector_final's __main__ does it. Watch the console for [TX] lines
    #    and the state prints; open http://localhost:5000 to see the overlay.
    worker = threading.Thread(target=det.control_loop, daemon=True)
    worker.start()

    print("[harness] control loop running. Open http://localhost:5000 to watch.")
    det.app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)