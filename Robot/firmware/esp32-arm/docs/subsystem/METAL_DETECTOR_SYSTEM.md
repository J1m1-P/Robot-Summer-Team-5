# Arm Metal Detector System

## Overview

The metal detector is a pulse-oscillator sensor wired to the arm ESP32. The
ESP32's hardware pulse counter (PCNT) counts falling edges on `PIN_METAL_DETECTOR`
over a fixed sample window, normalizes the count by the actual elapsed time,
and compares that frequency against a previously sampled baseline. A large
enough deviation from the baseline is reported as a detection.

The action controller combines detector samples with the rock lift and claw
servos. The drivetrain sends two commands after approaching a rock:

1. `CMD_METAL_SET_BASELINE` -- semi-lower the arm, open the claw, and capture
   the no-metal reference.
2. `CMD_METAL_READ` -- fully lower the arm, close the claw to center the rock,
   and then sample it. If metal is detected, fully lift it and keep the claw
   closed. Otherwise, open the claw at ground level, lift to the semi-lowered
   position, close the empty claw, and then fully lift.

## Hardware

| Item | Value |
|---|---|
| Pin | `PIN_METAL_DETECTOR` in `include/config/pin_map.h` (GPIO 45) |
| Counter | ESP32 PCNT unit 0, channel 0, falling edges only |
| Sample window | 100 ms (`METAL_DETECTOR_SAMPLE_PERIOD_MS`) |
| Glitch filter | 1 us (rejects sub-microsecond noise on the line) |
| Detection threshold | 1.5% of the baseline count (`METAL_DETECTOR_DETECT_THRESHOLD`) |

Tuning constants live in `src/config/metal_detector_config.c`. The detector
is a beat-frequency-style oscillator: its coil frequency is much higher than
what's actually counted, so treat the pulse counts empirically rather than
computing them from a datasheet frequency.

## Files

| File | Role |
|---|---|
| `include/drivers/metal_detector_driver.h` / `src/drivers/metal_detector_driver.c` | Owns the PCNT hardware: init, start/stop, fresh-window sampling, elapsed-time normalization, and baseline comparison. No knowledge of UART or command opcodes. |
| `include/config/metal_detector_config.h` / `src/config/metal_detector_config.c` | Pin, PCNT limits, sample window, and threshold tuning. |
| `include/control/task/metal_detector_action_controller.h` / `.cpp` | Runs the staged rock-servo and detector workflow, then queues the completion reply. |
| `src/main.cpp` | Initializes and starts the detector driver; the action controller captures the pose-correct baseline. |
| `src/control/task/arm_action_dispatcher.cpp` | Routes the two metal detector opcodes to the action controller alongside Tower/Habitat/Pi. |

## Wire Protocol

- `CMD_METAL_SET_BASELINE` -- no value payload. Completion is sent after the
  arm and claw settle and the baseline sample succeeds, with detail
  `STATUS_DETAIL_METAL_BASELINE_SET`.
- `CMD_METAL_READ` -- no value payload. Completion is sent after the arm is
  fully lifted, with detail `STATUS_DETAIL_METAL_DETECTED` if the rock remains
  held by the closed claw, else `STATUS_DETAIL_METAL_NOT_DETECTED` after the
  rock is released and the empty claw retracts.
- PCNT errors, a zero-pulse sample, or a missing baseline reply `STATUS_FAULT`.
  Other arm commands remain available.

## Known Limitations

- **Not yet wired into any sequence.** `robot_sequence_controller.c` does not
  send either opcode today, but its completion logic accepts both explicit
  read outcomes when the two steps are added after a rock approach.
- **GPIO 45 is an ESP32-S3 strapping pin.** Confirm the detector output is not
  driving an unsafe strap level during reset, or move the signal to a wiring-
  approved non-strapping GPIO.
- **`PIN_SOLAR_PANEL_MICROSWITCH` is currently unassigned** (`GPIO_NUM_NC`)
  in `pin_map.h` pending a wiring decision -- it used to share GPIO 45 with
  the metal detector. `arm_action_dispatcher.cpp` still calls `pinMode`/
  `digitalRead` on it every loop tick, so that needs a real pin before the
  solar-panel contact status is usable again.
