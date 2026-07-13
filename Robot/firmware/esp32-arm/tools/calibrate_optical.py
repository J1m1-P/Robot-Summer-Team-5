#!/usr/bin/env python3
"""Guide optical-flow passes and print JSON-ready sensor matrices.

Flash `pio run -e calibration -t upload` first. The ESP32 publishes
CAL,L,dx,dy,valid and CAL,R,dx,dy,valid records over its USB serial port.
"""

import argparse
import json
import math
import re
import sys
import threading

try:
    import serial
except ImportError:
    sys.exit("Missing dependency: python -m pip install -r tools/requirements-calibration.txt")


CAL_RECORD = re.compile(r"^CAL,([LR]),(-?\d+),(-?\d+),([01])$")


class SerialCapture:
    def __init__(self, port, baud):
        self.serial = serial.Serial(port, baud, timeout=0.1)
        self.lock = threading.Lock()
        self.active = False
        self.totals = {"L": [0, 0], "R": [0, 0]}
        self.samples = {"L": 0, "R": 0}
        self.stop_reader = threading.Event()
        self.reader = threading.Thread(target=self._read, daemon=True)
        self.reader.start()

    def _read(self):
        while not self.stop_reader.is_set():
            try:
                raw = self.serial.readline()
            except serial.SerialException:
                # close() intentionally interrupts a blocking readline().
                if self.stop_reader.is_set():
                    return
                raise
            if not raw:
                continue
            match = CAL_RECORD.match(raw.decode("ascii", errors="replace").strip())
            if match is None or match.group(4) != "1":
                continue
            sensor, dx, dy = match.group(1), int(match.group(2)), int(match.group(3))
            with self.lock:
                if self.active:
                    self.totals[sensor][0] += dx
                    self.totals[sensor][1] += dy
                    self.samples[sensor] += 1

    def begin_pass(self):
        with self.lock:
            self.serial.reset_input_buffer()
            self.totals = {"L": [0, 0], "R": [0, 0]}
            self.samples = {"L": 0, "R": 0}
            self.active = True

    def end_pass(self):
        with self.lock:
            self.active = False
            return ({sensor: tuple(value) for sensor, value in self.totals.items()}, self.samples.copy())

    def close(self):
        self.stop_reader.set()
        self.reader.join(timeout=1.0)
        self.serial.close()


def unit(vector):
    magnitude = math.hypot(*vector)
    if magnitude < 1.0:
        raise ValueError("pass had too little motion; repeat it with a longer movement")
    return vector[0] / magnitude, vector[1] / magnitude


def axis_direction(vectors, sensor, axis):
    directions = [unit(vector) for vector in vectors]
    # A pass may be made in either direction. Align every pass with the first
    # one, which establishes this axis's sign convention for the session.
    reference = directions[0]
    aligned = []
    for direction in directions:
        alignment = direction[0] * reference[0] + direction[1] * reference[1]
        if abs(alignment) < 0.5:
            raise ValueError(f"{sensor} {axis} passes do not describe one axis")
        aligned.append(direction if alignment > 0.0 else (-direction[0], -direction[1]))

    average = unit((sum(v[0] for v in aligned), sum(v[1] for v in aligned)))
    return average


def collect_axis(capture, axis, passes):
    results = {"L": [], "R": []}
    for number in range(1, passes + 1):
        input(f"{axis} pass {number}/{passes}: press Enter to start recording... ")
        capture.begin_pass()
        input(f"Move smoothly along {axis} in either direction, then press Enter to stop... ")
        totals, samples = capture.end_pass()
        for sensor in ("L", "R"):
            if samples[sensor] == 0:
                raise ValueError(f"{sensor} received no valid samples during {axis} pass {number}")
            results[sensor].append(totals[sensor])
        print(f"  recorded L={totals['L']} R={totals['R']}")
    return results


def signed_degrees(angle):
    """Normalize an angle to [-180, 180) degrees for readable residuals."""
    return math.degrees(math.atan2(math.sin(angle), math.cos(angle)))


def rotation_matrix(x_direction, y_direction):
    """Return the closest unit-scale proper-rotation matrix and fit details."""
    # Y passes are also direction-agnostic. Select their sign so X/Y form a
    # right-handed coordinate frame, which is required by a proper rotation.
    if x_direction[0] * y_direction[1] - x_direction[1] * y_direction[0] < 0.0:
        y_direction = (-y_direction[0], -y_direction[1])

    x_angle = math.atan2(x_direction[1], x_direction[0])
    # A +Y pass must map to the second column of a proper rotation: (-sin, cos).
    y_angle = math.atan2(y_direction[1], y_direction[0]) - math.pi / 2.0
    sine_sum = math.sin(x_angle) + math.sin(y_angle)
    cosine_sum = math.cos(x_angle) + math.cos(y_angle)
    if math.hypot(sine_sum, cosine_sum) < 1.0:
        raise ValueError("X and Y directions are not consistent with a rotation-only sensor mount")

    angle = math.atan2(sine_sum, cosine_sum)
    cosine, sine = math.cos(angle), math.sin(angle)
    if (x_direction[0] * cosine + x_direction[1] * sine < 0.5 or
            y_direction[0] * -sine + y_direction[1] * cosine < 0.5):
        raise ValueError("X and Y directions are too far from a rotation-only fit")

    matrix = {
        "m00": round(cosine, 6),
        "m01": round(-sine, 6),
        "m10": round(sine, 6),
        "m11": round(cosine, 6),
    }
    return matrix, {
        "angle_degrees": signed_degrees(angle),
        "x_residual_degrees": signed_degrees(x_angle - angle),
        "y_residual_degrees": signed_degrees(y_angle - (angle + math.pi / 2.0)),
    }


def print_interpretation(name, matrix, fit):
    print(f"\n{name.capitalize()} sensor")
    print(f"  Rotation: {fit['angle_degrees']:+.2f} deg (raw axes relative to robot axes)")
    print("  Scale: 1.000000 (fixed; not calibrated)")
    print("  Sign convention: the first X pass defines positive forward")
    print(f"  Forward rotation axis: ({matrix['m00']:+.6f}, {matrix['m10']:+.6f})")
    print(f"  Lateral rotation axis: ({matrix['m01']:+.6f}, {matrix['m11']:+.6f})")
    print(f"  X/Y fit residuals: {fit['x_residual_degrees']:+.2f} deg, "
          f"{fit['y_residual_degrees']:+.2f} deg")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="serial port, e.g. COM5 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--passes", type=int, help="passes per axis; prompted when omitted")
    args = parser.parse_args()

    passes = args.passes or int(input("Enter number of passes per axis: "))
    if passes < 1:
        sys.exit("Pass count must be positive.")

    capture = SerialCapture(args.port, args.baud)
    try:
        print("Passes may move in either direction. The first X pass defines positive forward.")
        x_passes = collect_axis(capture, "X", passes)
        y_passes = collect_axis(capture, "Y", passes)

        left, left_fit = rotation_matrix(axis_direction(x_passes["L"], "left", "X"),
                                         axis_direction(y_passes["L"], "left", "Y"))
        right, right_fit = rotation_matrix(axis_direction(x_passes["R"], "right", "X"),
                                            axis_direction(y_passes["R"], "right", "Y"))
        output = {"left": left, "right": right}
    except (KeyboardInterrupt, ValueError) as error:
        sys.exit(f"\nCalibration cancelled: {error}")
    finally:
        capture.close()

    print("\nCopy these left/right entries into data/calibration.json:")
    print(json.dumps(output, indent=2))
    print("\nInterpretation (matrix maps robot motion to raw sensor counts):")
    print_interpretation("left", left, left_fit)
    print_interpretation("right", right, right_fit)


if __name__ == "__main__":
    main()
