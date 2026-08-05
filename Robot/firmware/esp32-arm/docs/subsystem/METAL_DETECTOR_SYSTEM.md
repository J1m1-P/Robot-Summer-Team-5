# Arm Metal Detector System

## Overview

The metal detector is a pulse-oscillator sensor wired to the arm ESP32. The
ESP32's hardware pulse counter (PCNT) counts falling edges on `PIN_METAL_DETECTOR`
over a fixed sample window, normalizes the count by the actual elapsed time,
and compares that frequency against a previously sampled baseline. A large
enough deviation from the baseline is reported as a detection.

The action controller combines detector samples with the rock lift and claw
servos. A read command performs the complete rock workflow:

1. Move the arm down and open the claw.
2. Capture a fresh no-metal baseline.
3. Close the claw, move to the read position, and sample the rock.
4. If metal is detected, keep the claw closed and lift to up.
5. If no metal is detected, move down, close the claw, and then lift to up.

After a positive detection, subsequent `CMD_METAL_READ` commands report the
positive result immediately and do not repeat the claw workflow. The latch is
cleared when the arm controller is reinitialized.


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
| `src/control/task/arm_action_dispatcher.cpp` | Routes the metal detector read command to the action controller alongside Tower/Habitat/Pi. |

## Wire Protocol

- `CMD_METAL_READ` -- no value payload. The command establishes a fresh
  baseline before sampling and completes after the arm is fully lifted. The
  claw is closed and settled before every movement to the up position.
- PCNT errors or a zero-pulse sample reply `STATUS_FAULT`.
  Other arm commands remain available.

## Known Limitations

- The drivetrain test sequence currently issues three stationary read commands
  with five-second delays between them.
- **GPIO 45 is an ESP32-S3 strapping pin.** Confirm the detector output is not
  driving an unsafe strap level during reset, or move the signal to a wiring-
  approved non-strapping GPIO.
- **`PIN_SOLAR_PANEL_MICROSWITCH` is currently unassigned** (`GPIO_NUM_NC`)
  in `pin_map.h` pending a wiring decision -- it used to share GPIO 45 with
  the metal detector. `arm_action_dispatcher.cpp` still calls `pinMode`/
  `digitalRead` on it every loop tick, so that needs a real pin before the
  solar-panel contact status is usable again.
