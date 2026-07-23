"""Run the stationary vision scan requested through the robot task system.

The process combines PiTaskServer with a nonblocking camera/YOLO scanner.
UART events start or cancel scanning, while scanner completion is translated
back into the shared task success and failure codes.
"""

from __future__ import annotations

import argparse
import os
import time
from pathlib import Path

import cv2
from ultralytics import YOLO

from uart_link import (
    PiTaskServer,
    TASK_FAILURE_EXECUTOR_UNAVAILABLE,
    TASK_FAILURE_TARGET_NOT_FOUND,
    TASK_STEP_CANCELLED,
    TASK_STEP_FAILED,
    TASK_STEP_SUCCEEDED,
)


class VisionScanner:
    """Own camera/model resources and advance one time-bounded scan."""

    def __init__(self, model_path: Path, camera_index: int, image_size: int,
                 confidence: float, scan_seconds: float,
                 targets_required: int) -> None:
        """Load ``model_path`` and open ``camera_index`` with scan parameters."""
        self.image_size = image_size  # Square model inference resolution.
        self.confidence = confidence  # Minimum accepted YOLO confidence.
        self.scan_seconds = scan_seconds  # Maximum duration of one command.
        self.targets_required = targets_required  # Distinct labels for success.
        self.deadline = 0.0  # Monotonic end time for the active scan.
        self.seen: set[str] = set()  # Distinct detected class names.
        self.running = False  # True only while update() should infer frames.
        self.error: Exception | None = None  # Persistent initialization error.
        self.model = None  # Loaded Ultralytics YOLO model.
        self.camera = None  # OpenCV capture owned by this scanner.
        try:
            self.model = YOLO(str(model_path), task="detect")
            self.camera = cv2.VideoCapture(camera_index)
            if not self.camera.isOpened():
                raise RuntimeError(f"camera {camera_index} could not be opened")
        except Exception as exc:  # deployment errors become protocol failures
            self.error = exc

    def start(self) -> bool:
        """Reset detections and begin a scan when resources are available."""
        self.seen.clear()
        self.deadline = time.monotonic() + self.scan_seconds
        self.running = self.error is None
        return self.running

    def cancel(self) -> None:
        """Stop inference without releasing reusable camera/model resources."""
        self.running = False

    def update(self) -> tuple[int, int] | None:
        """Process one frame and return a terminal task result when finished."""
        if not self.running:
            return None
        ok, frame = self.camera.read()
        if not ok:
            self.running = False
            return TASK_STEP_FAILED, TASK_FAILURE_EXECUTOR_UNAVAILABLE

        results = self.model.predict(frame, imgsz=self.image_size,
                                     conf=self.confidence, verbose=False)
        names = results[0].names
        for class_id in results[0].boxes.cls.tolist():
            self.seen.add(str(names[int(class_id)]))
        if len(self.seen) >= self.targets_required:
            self.running = False
            return TASK_STEP_SUCCEEDED, 0
        if time.monotonic() >= self.deadline:
            self.running = False
            return TASK_STEP_FAILED, TASK_FAILURE_TARGET_NOT_FOUND
        return None

    def close(self) -> None:
        """Release the camera resource during process shutdown."""
        if self.camera is not None:
            self.camera.release()


def parse_args() -> argparse.Namespace:
    """Parse serial, model, camera, inference, and completion settings."""
    default_model = Path(__file__).resolve().parent.parent / "best_ncnn_model"
    parser = argparse.ArgumentParser()
    parser.add_argument("--serial-port",
                        default=os.getenv("ROBOT_SERIAL_PORT", "/dev/serial0"))
    parser.add_argument("--baud", type=int,
                        default=int(os.getenv("ROBOT_SERIAL_BAUD", "115200")))
    parser.add_argument("--model-path", type=Path,
                        default=Path(os.getenv("ROBOT_MODEL_PATH", default_model)))
    parser.add_argument("--camera", type=int,
                        default=int(os.getenv("ROBOT_CAMERA_INDEX", "0")))
    parser.add_argument("--image-size", type=int, default=320)
    parser.add_argument("--confidence", type=float, default=0.60)
    parser.add_argument("--scan-seconds", type=float, default=15.0)
    parser.add_argument("--targets", type=int, default=4)
    return parser.parse_args()


def main() -> None:
    """Run the task-server and scanner loop until interrupted."""
    args = parse_args()
    link = PiTaskServer(args.serial_port, args.baud)
    scanner = VisionScanner(args.model_path, args.camera, args.image_size,
                            args.confidence, args.scan_seconds, args.targets)
    if scanner.error is not None:
        print(f"vision unavailable: {scanner.error}", flush=True)

    try:
        while True:
            for event in link.update():
                if event == "cancel":
                    scanner.cancel()
                elif event == "start" and not scanner.start():
                    link.complete(TASK_STEP_FAILED,
                                  TASK_FAILURE_EXECUTOR_UNAVAILABLE)

            result = scanner.update()
            if result is not None:
                link.complete(*result)
            time.sleep(0.001)
    except KeyboardInterrupt:
        scanner.cancel()
        link.complete(TASK_STEP_CANCELLED)
    finally:
        scanner.close()
        link.close()


if __name__ == "__main__":
    main()
