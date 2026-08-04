# Arm Metal Detector System

## Overview

The metal detector is a pulse-oscillator sensor wired to the arm ESP32. The
ESP32's hardware pulse counter (PCNT) counts falling edges on `PIN_METAL_DETECTOR`
over a fixed sample window, normalizes the count by the actual elapsed time,
and compares that frequency against a previously sampled baseline. A large
enough deviation from the baseline is reported as a detection.

The detector answers one-shot commands sent from the drivetrain over the
shared arm UART link. Each command starts a fresh measurement window; command
completion is delayed until that window closes. Usage is always the same
three-step sequence:

1. `CMD_METAL_SET_BASELINE` -- capture the current pulse count as the
   no-metal reference, with metal away from the sensor.
2. Move the arm/detector into position.
3. `CMD_METAL_READ` -- sample again and report whether the new frequency
   deviates from the baseline by more than the detection threshold.

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
| `include/control/task/metal_detector_action_controller.h` / `.cpp` | Translates `CMD_METAL_SET_BASELINE` / `CMD_METAL_READ` into driver calls and queues the `STATUS_ACTION_COMPLETE` reply. |
| `src/main.cpp` | Initializes and starts the driver at boot, then captures one complete nonzero startup baseline. |
| `src/control/task/arm_action_dispatcher.cpp` | Routes the two metal detector opcodes to the action controller alongside Tower/Habitat/Pi. |

## Wire Protocol

- `CMD_METAL_SET_BASELINE` -- no value payload. Reply: `STATUS_ACTION_COMPLETE`,
  detail `STATUS_DETAIL_METAL_BASELINE_SET`.
- `CMD_METAL_READ` -- no value payload. Reply: `STATUS_ACTION_COMPLETE`, detail
  `STATUS_DETAIL_METAL_DETECTED` if metal was found, else
  `STATUS_DETAIL_METAL_NOT_DETECTED`.
- PCNT errors, a zero-pulse sample, or a missing baseline reply `STATUS_FAULT`.
  Other arm commands remain available.

## Known Limitations

- **Not yet wired into any sequence.** `robot_sequence_controller.c` does not
  send either opcode today, but its completion logic accepts both explicit
  read outcomes when a step is added.
- **GPIO 45 is an ESP32-S3 strapping pin.** Confirm the detector output is not
  driving an unsafe strap level during reset, or move the signal to a wiring-
  approved non-strapping GPIO.
- **`PIN_SOLAR_PANEL_MICROSWITCH` is currently unassigned** (`GPIO_NUM_NC`)
  in `pin_map.h` pending a wiring decision -- it used to share GPIO 45 with
  the metal detector. `arm_action_dispatcher.cpp` still calls `pinMode`/
  `digitalRead` on it every loop tick, so that needs a real pin before the
  solar-panel contact status is usable again.
