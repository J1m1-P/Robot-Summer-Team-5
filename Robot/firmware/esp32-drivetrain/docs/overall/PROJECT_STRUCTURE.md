# Project Structure

The drivetrain firmware is split by responsibility so the production entry
point remains small and control code can be tested independently.

```text
esp32-drivetrain/
|-- include/
|   |-- comm/              Packet caches and link-facing helpers
|   |-- config/            Board wiring, geometry, gains, and limits
|   |-- control/
|   |   |-- drivetrain/    Drivetrain facade, kinematics, odometry
|   |   |-- line_following/
|   |   |-- motion/
|   |   |-- odometry/      Shared fused-pose service
|   |   |-- pid/
|   |   `-- task/          Movement actions and robot sequence
|   `-- drivers/           Motor, encoder, tape, and ToF hardware access
|-- src/
|   |-- comm/
|   |-- config/
|   |-- control/
|   |-- drivers/
|   |-- harnesses/         Alternative diagnostic entry points
|   `-- main.cpp           Production ownership, setup, and scheduling
|-- test/                  Native unit tests and hardware stubs
|-- tools/                 Browser diagnostics
|-- docs/                  Current subsystem and architecture notes
`-- platformio.ini         Build environments and source selection
```

## Layers

| Layer | Owns |
|---|---|
| Application | Object lifetime, startup order, update scheduling, and fault policy |
| Task and motion control | Robot steps, blocking maneuvers, stop conditions, and pose targets |
| Drivetrain control | Four-wheel coordination, limits, watchdog, wheel PI, and motor output |
| Drivers | GPIO/peripheral operations for one hardware family |
| Configuration | Immutable board-specific values |

Dependencies point downward. Drivers never call task or application code.

## Production lifecycle

`src/main.cpp`:

1. Holds motor outputs safe.
2. Initializes the arm UART, drivetrain, tape modules, and pose tracker.
3. Connects `PoseService`, `LineFollowerContext`, and
   `PrecisionMoveContext`.
4. Starts the robot sequence after the arm reports ready.
5. Services idle drivetrain, UART, pose, and sequence updates.

When a blocking maneuver runs, it temporarily owns drivetrain and
`PoseService` updates. The Arduino loop resumes after that maneuver returns.

## Where new code belongs

| New code | Location |
|---|---|
| Production composition or scheduling | `src/main.cpp` |
| Diagnostic firmware | `src/harnesses/*_main.cpp` plus a PlatformIO environment |
| Reusable control behavior | `control/<subsystem>/` |
| Direct peripheral access | `drivers/<device>/` |
| Board pins, gains, or limits | `config/<subsystem>/` |
| Native unit tests | `test/test_<module>/` |
| Shared code used by both ESP32 projects | `lib/robot-common/` |

Do not place pin numbers in generic drivers, hardware access in pure control
math, or reusable subsystem behavior in `main.cpp`.
