# Arm Metal Detector System

## Overview

The metal detector is a pulse-oscillator sensor wired to the arm ESP32. The
ESP32's hardware pulse counter (PCNT) counts falling edges on `PIN_METAL_DETECTOR`
over a fixed sample window and compares that count against a previously
sampled baseline. A large enough deviation from the baseline is reported as
a detection.

The detector does not track continuously -- it answers one-shot commands
sent from the drivetrain over the shared arm UART link. Usage is always the
same three-step sequence:

1. `CMD_METAL_SET_BASELINE` -- capture the current pulse count as the
   no-metal reference, with metal away from the sensor.
2. Move the arm/detector into position.
3. `CMD_METAL_READ` -- sample again and report whether the new count
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
| `include/drivers/metal_detector_driver.h` / `src/drivers/metal_detector_driver.c` | Owns the PCNT hardware: init, start/stop, paced sampling, baseline comparison. No knowledge of UART or command opcodes. |
| `include/config/metal_detector_config.h` / `src/config/metal_detector_config.c` | Pin, PCNT limits, sample window, and threshold tuning. |
| `include/control/task/metal_detector_action_controller.h` / `.cpp` | Translates `CMD_METAL_SET_BASELINE` / `CMD_METAL_READ` into driver calls and queues the `STATUS_ACTION_COMPLETE` reply. |
| `src/main.cpp` | Initializes and starts the driver at boot, then takes one priming read so the baseline is never left at zero before the first `CMD_METAL_SET_BASELINE`. |
| `src/control/task/arm_action_dispatcher.cpp` | Routes the two metal detector opcodes to the action controller alongside Tower/Habitat/Pi. |

The identical driver also lives standalone in `firmware/esp32-detector-test/`
for bench testing without the rest of the arm firmware.

## Wire Protocol

- `CMD_METAL_SET_BASELINE` -- no value payload. Reply: `STATUS_ACTION_COMPLETE`,
  detail `STATUS_DETAIL_NONE`.
- `CMD_METAL_READ` -- no value payload. Reply: `STATUS_ACTION_COMPLETE`, detail
  `STATUS_DETAIL_METAL_DETECTED` if metal was found, else `STATUS_DETAIL_NONE`.
- If the driver failed to initialize (missing/unwired sensor), both commands
  reply `STATUS_FAULT` instead, and the rest of the arm's commands keep
  working normally.

## Known Limitations

- **Not yet wired into any sequence.** `robot_sequence_controller.c` doesn't
  send either opcode today. If a sequence step is added around
  `CMD_METAL_READ`, note that `arm_action_status_detail()` has no case for it
  (falls through to `STATUS_DETAIL_NONE`) -- the sequence controller's
  exact-match step-completion check needs to accept either detail value for
  that opcode, or a detection reply will never be recognized as completing
  the step.
- **`PIN_SOLAR_PANEL_MICROSWITCH` is currently unassigned** (`GPIO_NUM_NC`)
  in `pin_map.h` pending a wiring decision -- it used to share GPIO 45 with
  the metal detector. `arm_action_dispatcher.cpp` still calls `pinMode`/
  `digitalRead` on it every loop tick, so that needs a real pin before the
  solar-panel contact status is usable again.
