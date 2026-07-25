# Robot Sequence Editing Workflow

## Original objective

**Verified:** Make mixed drivetrain/tape and Tower behavior easy to edit:

- Change robot behavior order in one sequence table.
- Implement each action in its corresponding action controller.
- Keep every task controller non-blocking and expose `init()` and `update()`.

The design evolved from separate Tape and Tower sequences into one drivetrain-side
`RobotSequenceController`. Tape-guided and ordinary drivetrain actions were combined
under `MovementActionController`.

## Current implementation status

**Verified:** The drivetrain owns the complete ordered sequence in:

`Robot/firmware/esp32-drivetrain/src/control/task/robot_sequence_controller.c`

The current order is:

1. Tape follow for 1 m.
2. Rotate clockwise until aligned with tape.
3. Run all configured Tower actions.
4. Go forward for 1 m.
5. Rotate 90 degrees.

Movement actions are placeholders that print once and immediately report completion.
Tower actions execute actual servo or stepper behavior on the arm ESP.

## Editing workflow

### Change the robot sequence

Edit only `kRobotSequence` in:

`Robot/firmware/esp32-drivetrain/src/control/task/robot_sequence_controller.c`

Each row contains:

```c
{step_type, {.action_group = ACTION_NAME}, action_value},
```

Movement example:

```c
{ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_GO_FORWARD}, 1.0f},
{ROBOT_STEP_MOVEMENT, {.movement = MOVEMENT_ACTION_ROTATE}, 90.0f},
```

Tower example:

```c
{ROBOT_STEP_ARM, {.arm = CMD_TOWER_OPEN_CLAW}, 0.0f},
{ROBOT_STEP_ARM, {.arm = CMD_TOWER_Z_UP}, 0.50f},
```

Existing actions may be reordered, repeated, removed, or assigned different values
without changing either action controller.

Keep action names behavior-based. Do not use names such as `STEP_3`, because an
action's meaning must not depend on its position in the sequence.

### Implement movement or tape-following behavior

The public action list and controller state are in:

`Robot/firmware/esp32-drivetrain/include/control/task/movement_action_controller.h`

The implementations are in:

`Robot/firmware/esp32-drivetrain/src/control/task/movement_action_controller.c`

Implement the matching `switch` case in
`movement_action_controller_update()`. Use
`movement_action_controller_init()` to validate parameters, initialize persistent
state, and start an action when appropriate.

Required update behavior:

- Return `false` while motion is still running.
- Return `true` only when the requested motion is complete.
- Avoid blocking loops and long delays; `update()` is called repeatedly from the
  main robot loop.
- Stop the drivetrain safely when an action completes or faults.

Current movement values are intended as:

- Tape-follow and forward distance: metres.
- Rotation: degrees; the signed-direction convention still needs to be defined when
  rotation is implemented.

### Edit Tower behavior

Tower controller state and interface:

`Robot/firmware/esp32-arm/include/control/task/tower_action_controller.h`

Tower behavior:

`Robot/firmware/esp32-arm/src/control/task/tower_action_controller.cpp`

For an existing Tower command, edit its case in `start_tower_action()`. Timed servo
actions set `action_complete_ms`; stepper actions set `active_stepper`.
`tower_action_controller_update()` detects completion and sends
`STATUS_ACTION_COMPLETE` back to the drivetrain ESP.

The current Tower distance conversion is:

```text
command value × 100 mm
```

For example, `0.50f` requests 50 mm.

Do not move Tower hardware behavior into `robot_sequence_controller.c`. The sequence
controller should only select the command, provide its value, wait for the matching
completion status, and advance.

### Add a completely new action

For a new movement action:

1. Add a descriptive `MOVEMENT_ACTION_*` value before `MOVEMENT_ACTION_MAX`.
2. Add its implementation to `movement_action_controller_update()`.
3. Add any persistent state needed by `MovementActionController`.
4. Add it to `kRobotSequence`.
5. Update the robot-sequence test.

For a new Tower action:

1. Add a shared `CMD_TOWER_*` opcode in the robot-common command protocol.
2. Add its completion-detail mapping in the shared status protocol.
3. Implement it in `start_tower_action()`.
4. Ensure `tower_action_controller_update()` can detect its completion.
5. Add it to `kRobotSequence` and update tests.

## Important decisions and reasoning

- **Verified:** `RobotSequenceController` is the single owner of cross-ESP action
  ordering.
- **Verified:** Action controllers own implementation details and completion state.
- **Verified:** Action names describe reusable behavior, not sequence positions.
- **Verified:** Arm steps advance only after a completion packet whose detail matches
  the active Tower command.
- **Verified:** Every robot step has a 15-second timeout.
- **Verified:** The arm repeats completion status temporarily to reduce the chance of
  a lost UART acknowledgement.

## Repository state

- Repository root: `C:/Personal/Projects/Robot/RobotSummer`
- Branch: `StepDriverFix`
- HEAD: `baa369e77cb641e8a33576cd817915886ea67afb`
- Subject: `JP: Generalized movement`
- **Verified before creating this document:** staged, unstaged, and untracked state
  was clean.
- This `HANDOFF.md` is the only file created for this documentation request.

## Files changed

- `HANDOFF.md` — new workflow and handoff document.
- No source or configuration files were changed while preparing this handoff.

## Commands and tests actually run

Previously evidenced during implementation:

- `platformio test -e robot-sequence-native`
- `platformio test -e native`
- `platformio run -e esp32-s3-devkitm-1` from `esp32-drivetrain`
- `platformio run -e esp32-s3-devkitm-1` from `esp32-arm`
- `git diff --check`
- `rg` searches for stale Tape controller and `STEP_3` names

Read-only handoff inspection:

- `git rev-parse --show-toplevel`
- `git --no-optional-locks status --short --branch`
- `git branch --show-current`
- `git rev-parse HEAD`
- `git log -1`
- `git diff --stat`
- `git diff --cached --stat`
- `git ls-files --others --exclude-standard`

## Test results

- **Verified:** Updated robot-sequence tests passed: 3/3.
- **Verified:** Drivetrain native regression suite passed: 127/127.
- **Verified:** Production drivetrain firmware built successfully.
- **Verified:** Production arm firmware built successfully.
- **Verified:** No stale Tape action-controller, `STEP_3`, or `ROBOT_STEP_TAPE`
  references remained after the rename.
- No new tests were run solely to prepare this handoff, as required by the handoff
  workflow.

## Known issues and uncertainties

- **Verified:** Movement actions are still immediate-completion placeholders.
- **Unknown:** Actual movement behavior, physical UART communication, and complete
  robot sequencing have not been validated on two flashed ESPs in this session.
- **Unknown:** The sign convention for `MOVEMENT_ACTION_ROTATE` has not been defined.
- **Verified:** The 15-second timeout may need adjustment for real actions that
  legitimately take longer.

## Unfinished work

1. Implement all four movement actions using the existing drivetrain and tape modules.
2. Define and document the signed rotation convention.
3. Replace immediate placeholder completion with persistent non-blocking state.
4. Add controller-level tests for movement initialization, running, completion, and
   faults.
5. Flash both ESPs and run an end-to-end hardware sequence test.

## Exact recommended next steps

1. Read the drivetrain and tape-following public interfaces under
   `esp32-drivetrain/include/control/`.
2. Extend `MovementActionController` with only the state needed to track one active
   action.
3. Implement `MOVEMENT_ACTION_GO_FORWARD` first using a non-blocking drivetrain move.
4. Add native tests for initialization, repeated updates, exact completion, and safe
   stop behavior.
5. Implement rotation and the two tape-guided actions using the same pattern.
6. Run:

```text
cd Robot/firmware/esp32-drivetrain
platformio test -e robot-sequence-native
platformio test -e native
platformio run -e esp32-s3-devkitm-1

cd ../esp32-arm
platformio run -e esp32-s3-devkitm-1
```

7. Flash both ESPs, verify crossed UART TX/RX wiring, and test the sequence with the
   robot safely elevated before floor testing.

## Continuation prompt

```text
Continue the robot action-controller work in
C:/Personal/Projects/Robot/RobotSummer.

Verified state:
- Branch StepDriverFix at baa369e77cb641e8a33576cd817915886ea67afb
  ("JP: Generalized movement").
- RobotSequenceController owns the mixed movement/Tower order.
- MovementActionController contains tape-follow, tape-align, forward, and rotate
  actions, but they are immediate-completion placeholders.
- TowerActionController executes arm behavior and reports command-specific completion
  over UART.
- Sequence tests passed 3/3, native drivetrain tests passed 127/127, and both
  production firmware targets built successfully during the prior implementation.

Objective:
Implement movement actions as concise, non-blocking init/update state machines while
keeping robot_sequence_controller.c limited to action order and values.

Constraints:
- Preserve the action-controller separation.
- Use behavior-based action names, never position-based names such as STEP_3.
- Return false while an action is running and true only when complete.
- Safely stop the drivetrain on completion or fault.
- Preserve matching arm-command acknowledgements and the sequence timeout.
- Inspect the current repository and working tree before editing because they may
  have changed since this handoff was written.

After implementation, update relevant tests, run both drivetrain test environments,
build both ESP production targets, and report any hardware validation that remains.
```
