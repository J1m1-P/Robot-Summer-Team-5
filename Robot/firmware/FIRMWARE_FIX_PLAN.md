# Firmware Issue Summary & Fix Plan

> Output of a design review of `Robot/firmware` (2026-07-13). Covers both PlatformIO
> projects: `esp32-drivetrain` and `esp32-arm`. Fix work is organized into four
> ordered sessions; deferred issues are listed at the bottom so they don't get lost.

---

## Issue summary

### Fix now (Critical + High)

| # | Issue | Where |
|---|-------|-------|
| 1 | `motor_driver_init` returns `bool` but is checked as `esp_err_t` — drivetrain init logic is inverted, can never succeed | `esp32-drivetrain/include/drivers/motor_driver.h:39`, `src/control/drivetrain.c:76` |
| 2 | `LOG_TAG_UART` missing from drivetrain's tag table → NULL tag → likely crash on first UART error log | `esp32-drivetrain/src/debug/app_log.c:3-9` |
| 3 | `calibration.json` missing `baseline_mm` → calibration loader always fails, fusion dead | `esp32-arm/data/calibration.json` |
| 4 | Arm "production" main.cpp is the empty Arduino stub; drivetrain has no odometry packet consumer — the inter-board link has never run | `esp32-arm/src/main.cpp`, empty `odometry_packet.*` files |
| 5 | `uart_link` / `app_log` / packet types copy-pasted between projects, already diverged — protocol drift risk | both projects |
| 6 | Stepper driver blocks the CPU for seconds per move (kills UART + sensor polling); circular include; `static` declarations in header; duplicated pins | `esp32-arm` stepper files |
| 7 | No failsafe: robot keeps driving last duty forever if commands stop; brake released as side effect of set-duty | `drivetrain.c` |
| 8 | Error-code inconsistencies (`bool` vs `esp_err_t`, `INVALID_ARG` vs `INVALID_STATE`, pmw3610 init can't report failure) | drivers |
| 9 | Silent failures: calibration loader has ~8 unlogged `return false` paths; uart_link error counters never surfaced | `static_calibration.cpp`, `uart_link.c` |

### Deferred (Medium/Low — solve later, when touching these modules)

- **M1** Encoder: PCNT limit wrap has no ISR (counts silently lost past ±32000 between updates); pause/clear/resume in `flush_delta` loses edges every update (systematic undercount).
- **M2** Motor: PWM polarity mystery (commented-out `1.0f - duty` vs. "HIGHER = SLOWER" comment) — **resolve on hardware early, it's a 5-minute test**; all motors share `LEDC_TIMER_0` and each init reconfigures it.
- **M3** UART: single-packet mailbox drops messages under burst (mitigated if odometry packet becomes cumulative — see Session 2 step 6).
- **M4** Tape module: `sensor_0..3` should be an array; `pin_is_on_tape` naming; PID weight configs (`FRONT_PID_WEIGHTS` etc.) not declared in any header; 5 µs mux settle needs a scope check.
- **M5** I2C: accepts reserved addresses 0x00–0x07 in `i2c_device_init`; no stuck-bus recovery; no mutex — **add the mutex before ToF polling moves to a task**.
- **M6** Units diverging (drivetrain = m, arm = mm) — partially handled at the odometry boundary in Session 3.
- **M7** No written concurrency rule — covered by the conventions doc in Session 4.
- Host-side unit tests for pure modules (PID, kinematics, parser, fusion, pose) — high value, one weekend.
- `upload_port = COM11` hardcoded in drivetrain `platformio.ini` (breaks other machines' checkouts).
- ESP32-S3 strapping-pin review (encoder on GPIO 45/46, mux selects on 38/39).
- Stale comments/typos in config files (`MOTOR_MAX_DUTY 1.0f // 80%`, `RESOLURION`, encoder CPR example vs. actual); delete empty placeholder files; style-align stepper module.
- `MAX_DUTY 0.4` in `drivetrain_config.c` — if it's a bring-up limit, mark it `// TODO raise after tuning`.

---

## Session 1 — Kill the critical bugs (~1 hour)

### 1a. Fix `motor_driver_init` return type

In `motor_driver.h` change the declaration:

```c
esp_err_t motor_driver_init(MotorDriver *motor, const MotorDriverConfig *config);
```

In `motor_driver.c` update the body — return real error codes instead of `false` so the caller's log shows the cause:

```c
esp_err_t motor_driver_init(MotorDriver *motor, const MotorDriverConfig *config) {
    if (motor == NULL || config == NULL) return ESP_ERR_INVALID_ARG;
    if (config->max_duty < 0.0f || config->max_duty > 1.0f) return ESP_ERR_INVALID_ARG;
    if (config->pwm_resolution == 0 || config->pwm_resolution > 20) return ESP_ERR_INVALID_ARG;
    if (config->pwm_frequency == 0) return ESP_ERR_INVALID_ARG;
    if (config->pwm_channel >= LEDC_CHANNEL_MAX) return ESP_ERR_INVALID_ARG;

    /* ... */

    esp_err_t err = gpio_config(&dir_gpio_config);
    if (err != ESP_OK) return err;

    err = ledc_timer_config(&pwm_timer_config);
    if (err != ESP_OK) return err;

    err = ledc_channel_config(&pwm_channel_config);
    if (err != ESP_OK) return err;

    /* ... */
    return ESP_OK;
}
```

The call site in `drivetrain.c:76` is then already correct as written. No other callers exist.

### 1b. Fix the NULL log tag

In `esp32-drivetrain/src/debug/app_log.c`, add the missing entry **and** a defensive
NULL check so this class of bug can't recur:

```c
static const char *LOG_TAGS[LOG_TAG_MAX] = {
    [LOG_TAG_MAIN] = "main",
    [LOG_TAG_MOTOR] = "motor",
    [LOG_TAG_ENCODER] = "encoder",
    [LOG_TAG_DRIVETRAIN] = "drivetrain",
    [LOG_TAG_UART] = "uart",
    [LOG_TAG_I2C] = "i2c"
};

const char *app_log_tag(LogTag tag) {
    if (tag < 0 || tag >= LOG_TAG_MAX || LOG_TAGS[tag] == NULL) {
        return "unknown";
    }
    return LOG_TAGS[tag];
}
```

Apply the same NULL-check pattern to the arm's copy (its table is complete today, but
the guard is free insurance until Session 2 merges the two files anyway).

### 1c. Fix `calibration.json`

Measure the sensor separation with calipers and add the key:

```json
{
  "baseline_mm": <your measured value>,
  "left":  { "m00": 0.999877, "m01": 0.015658, "m10": -0.015658, "m11": 0.999877 },
  "right": { "m00": 0.999157, "m01": 0.041044, "m10": -0.041044, "m11": 0.999157 }
}
```

Then re-upload the filesystem: `pio run -e optical_debug -t uploadfs`.

### 1d. Make calibration failures loud

In `static_calibration.cpp`, add an `APP_LOGE(LOG_TAG_FUSION, ...)` to each
`return false` branch — mount failed, file missing, JSON parse error (include
`error.c_str()`), bad baseline, bad left matrix, bad right matrix, not-rotation-only.
Seven log lines; this is what would have caught 1c immediately.

### ✅ Verify Session 1

- Flash `optical_debug` — you should see the fused pose stream instead of
  "fusion disabled".
- Flash the drivetrain with a temporary main that calls
  `drivetrain_init(&dt, &DRIVETRAIN_CONFIG)` and prints the result — it should return
  `ESP_OK` for the first time.
- While the motors are hooked up anyway: **run the PWM-polarity test (M2)** —
  confirm whether higher duty = faster or slower on the real driver board, fix the
  code or the comment accordingly, and record the answer in the code.

---

## Session 2 — Shared `common/` library (one evening)

Do this **before** writing any main.cpp integration code, so the integration is
written against the shared protocol once, not twice.

### Target structure

```
Robot/firmware/
├── common/
│   ├── library.json
│   ├── include/
│   │   ├── comm/uart_link.h
│   │   ├── comm/packets/delta_pose_packet.h
│   │   ├── debug/app_log.h
│   │   └── robot/robot_types.h
│   └── src/
│       ├── uart_link.c
│       ├── delta_pose_packet.c
│       └── app_log.c
├── esp32-drivetrain/
└── esp32-arm/
```

### Steps

1. Copy the **arm's** versions of `uart_link.c/.h` and `delta_pose_packet.c/.h` into
   `common/` (arm's formatting is the cleaner of the two diverged copies; the logic is
   identical). Copy `robot_types.h` from the drivetrain.
2. `app_log`: the two boards have different tag sets — make the tag list the **union**
   of both enums (`MAIN, MOTOR, ENCODER, DRIVETRAIN, PMW3610, FUSION, UART, I2C, MAX`).
   Unused tags on a board cost nothing. Keep the NULL-check guard from 1b.
3. `library.json`:

   ```json
   { "name": "common", "version": "0.1.0", "build": { "srcDir": "src", "includeDir": "include" } }
   ```

4. In **both** `platformio.ini` files add under `[env]`:

   ```ini
   lib_extra_dirs = ../
   ```

5. Delete the per-project copies (`esp32-drivetrain/src/communication/uart/`,
   `esp32-drivetrain/src/debug/`, `esp32-arm/src/comm/`, `esp32-arm/src/debug/`, and
   their headers, plus the empty `odometry_packet.*` placeholders). Fix the
   drivetrain's includes: `communication/uart/uart_link.h` → `comm/uart_link.h`.
6. **One protocol decision:** change the odometry packet from per-cycle deltas to
   **cumulative pose** (`x_mm, y_mm, theta_rad, valid`) — move the mid-point-heading
   integration to run on the arm before sending. Then a dropped/corrupted packet costs
   ~10 ms of staleness instead of permanently lost distance, and the single-mailbox
   overwrite behavior (deferred issue M3) becomes *correct by design* — the newest
   cumulative pose is always the one you want. ~20 lines now; removes a whole failure
   class before the controller depends on it.

### ✅ Verify Session 2

- Both projects build: `pio run` in each.
- No duplicated protocol definitions remain:
  `grep -r PACKET_MAGIC Robot/firmware --include=*.c --include=*.h` hits only `common/`.

---

## Session 3 — End-to-end integration + failsafe (one evening, needs both boards wired)

### 3a. Real arm `main.cpp`

Replace the stub with the pipeline (all pieces already exist):

```cpp
#include <Arduino.h>
#include "comm/uart_link.h"
#include "comm/packets/delta_pose_packet.h"
#include "config/pin_map.h"
#include "config/static_calibration.h"
#include "config/uart_link_config.h"
#include "debug/app_log.h"
#include "drivers/pmw3610_driver.h"
#include "sensing/pmw3610_fusion.h"

static DualPmw3610 sensors;
static Pmw3610Fusion fusion;
static UartLink link = {0};
static bool fusion_ok = false;

void setup() {
    Serial.begin(115200);
    delay(1000);
    app_log_init();

    const PmwPinConfig pins = {PIN_PMW_SDIO, PIN_PMW_SCLK, PIN_PMW_NCS_L, PIN_PMW_NCS_R};
    dual_pmw3610_init(&sensors, &pins);

    FusionConfig cal;
    fusion_ok = static_calibration_load(&cal) && pmw3610_fusion_configure(&fusion, &cal);
    if (!fusion_ok) APP_LOGE(LOG_TAG_FUSION, "Calibration invalid -- sending valid=false");

    if (uart_link_init(&link, &DRIVETRAIN_UART_LINK_CONFIG) != ESP_OK) {
        APP_LOGE(LOG_TAG_UART, "UART link init failed");
    }
}

void loop() {
    int16_t ldx, ldy, rdx, rdy;
    bool l_valid, r_valid;
    dual_pmw3610_poll(&sensors, &ldx, &ldy, &l_valid, &rdx, &rdy, &r_valid);

    const DeltaPose delta = pmw3610_fusion_process(&fusion, ldx, ldy, rdx, rdy);
    const bool valid = fusion_ok && l_valid && r_valid;
    delta_pose_packet_send(&link, &delta, valid);

    delay(10);   // ~100 Hz; replace with paced loop in Session 4
}
```

(If you adopted cumulative pose in Session 2 step 6, integrate here before sending.)

### 3b. Drivetrain receiver

New module `esp32-drivetrain/src/robot/odometry.c` (+ header): consumes packets, owns
the drivetrain-side `RobotOdometry`. Core:

```c
esp_err_t odometry_update(RobotOdometry *odom, UartLink *link) {
    esp_err_t err = uart_link_update(link);
    if (err != ESP_OK) return err;

    PacketFrame frame;
    while (uart_link_take_packet(link, &frame) == ESP_OK) {
        DeltaPose delta;
        bool valid;
        if (delta_pose_packet_decode(&frame, &delta, &valid) != ESP_OK) continue;
        odom->valid = valid;
        odom->timestamp_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (!valid) continue;
        // mid-point heading integration (mm -> m at this boundary)
        float mid = odom->pose.theta_rad + delta.dtheta_rad * 0.5f;
        odom->pose.x_m += (delta.dx_mm * cosf(mid) - delta.dy_mm * sinf(mid)) / 1000.0f;
        odom->pose.y_m += (delta.dx_mm * sinf(mid) + delta.dy_mm * cosf(mid)) / 1000.0f;
        odom->pose.theta_rad += delta.dtheta_rad;
    }
    return ESP_OK;
}
```

The mm→m conversion happens **here, at the boundary** — everything drivetrain-side
stays SI (deferred issue M6 handled where it matters most).

### 3c. Failsafe in the drivetrain (do it now — it shapes the controller API)

In `Drivetrain`, add `int64_t last_command_us;`. Stamp it in
`drivetrain_set_motor_duty`, `drivetrain_set_all_motor_duty`, and
`drivetrain_set_body_duty`. Add:

```c
// Call every loop iteration. Coasts if no command arrived recently.
esp_err_t drivetrain_tick(Drivetrain *dt) {
    if (dt == NULL || !dt->initialized) return ESP_ERR_INVALID_STATE;
    if (!dt->enabled) return ESP_OK;
    int64_t now = esp_timer_get_time();
    if (now - dt->last_command_us > DRIVETRAIN_CMD_TIMEOUT_US) {  // e.g. 250000
        return drivetrain_coast(dt);
    }
    return ESP_OK;
}
```

Also pull the `gpio_set_level(brk_pin, 0)` out of `drivetrain_set_motor_duty` /
`set_all_motor_duty` into an explicit `drivetrain_release_brake()`, called from
`drivetrain_enable()` — brake state stops being a hidden side effect.

### ✅ Verify Session 3 — the acceptance gate

Wire the two boards' UART (arm 47/48 ↔ drivetrain 47/48, crossed, common ground).
Temporary drivetrain main: init uart_link + odometry, loop
`{ odometry_update(); print pose at 2 Hz; }`.

1. Slide the arm's sensor rig around by hand — the drivetrain's printed pose must
   track it.
2. Unplug the UART wire mid-run — confirm (a) no crash, (b) `valid` goes stale,
   (c) once motors are commanded, `drivetrain_tick` coasts them.

**When this passes, the two-board architecture is real.**

---

## Session 4 — Non-blocking stepper + loop skeleton (one evening)

### 4a. Stepper rework (issue 6, all four defects at once)

- Remove `#include "drivers/stepper_driver.h"` from `stepper_config.h` (breaks the
  include cycle — it doesn't need it).
- Remove the two `static` function declarations from `stepper_driver.h`.
- Replace the `inline` config variables with the house pattern:
  `extern const StepperConfig TOWER_X_CONFIG;` in the header, definitions in a
  `stepper_config.cpp` using `PIN_STEP1` / `PIN_STEP1_DIR` macros from `pin_map.h`
  (kills the duplicated pin numbers).
- Convert to the service pattern:

```c
typedef struct {
    uint8_t stepPin, dirPin;
    uint32_t stepPulseUs, stepDelayUs;
    long steps_remaining;      // 0 = idle
    long position_steps;       // signed cumulative position
    int8_t direction;          // +1 / -1
    int64_t next_step_us;
} StepperDriver;

void stepper_start_move(StepperDriver *d, long steps);   // sets dir pin, loads counter
bool stepper_is_moving(const StepperDriver *d);
void stepper_service(StepperDriver *d);                  // call every loop; <=1 pulse per call
```

`stepper_service`: check `esp_timer_get_time() >= next_step_us`, emit one ~3 µs pulse,
decrement, update `position_steps`, schedule the next. Keep `stepper_move_steps` as a
documented-blocking convenience for bench scripts if wanted, but the arm controller
only uses start/service/is_moving. This is also exactly the
`start()/update()/is_done()` shape every future arm motion primitive will follow.

One `loop()` iteration now services all four steppers → simultaneous X/Z moves work,
UART and sensors keep running.

### 4b. Fixed-rate loop skeleton on the drivetrain

Turn the Session 3 test main into the permanent structure the controller layer will
drop into:

```cpp
void loop() {
    static int64_t next_cycle_us = 0;
    int64_t now = esp_timer_get_time();
    if (now < next_cycle_us) return;
    next_cycle_us = now + 10000;              // 100 Hz

    odometry_update(&odom, &link);            // read inputs
    drivetrain_encoder_update(&dt);
    // <- state machine / controllers slot in here later
    drivetrain_tick(&dt);                     // failsafe, always last
}
```

### 4c. Conventions doc (issue 8, 30 minutes)

`Robot/firmware/README.md`, one page:

- **Errors** — all fallible functions return `esp_err_t`; `INVALID_ARG` = caller bug,
  `INVALID_STATE` = wrong lifecycle.
- **Units** — SI (m, rad, s) at every module boundary; convert at sensor/wire edges.
- **Concurrency** — one control task owns all hardware; all driver calls from `loop()`
  only.
- **Naming** — snake_case, `module_verb()`, `Config`/runtime struct pairs.

Then two quick alignments: pmw3610 `bus_init_sensor` → `esp_err_t` (fail on wrong
product ID / self-test), and swap the `INVALID_ARG`-for-uninitialized returns to
`INVALID_STATE` in motor/encoder/drivetrain.

---

## After these four sessions

- Both boards boot their real firmware.
- One shared protocol library; no duplicated protocol code.
- Odometry flows end-to-end with a tested failure mode.
- A failsafe that can't be forgotten (called from the loop skeleton).
- Steppers that don't freeze the arm board.
- The fixed-rate loop skeleton the state machine plugs into.

Nothing in the deferred list blocks controller work — with two exceptions to keep on
the radar:

1. **PWM-polarity hardware test (M2)** — do it the first time a motor spins
   (Session 1 verify). An inverted throttle is the most dangerous latent bug in the
   codebase.
2. **I2C bus mutex (M5)** — add it before anyone moves ToF polling into a task.
