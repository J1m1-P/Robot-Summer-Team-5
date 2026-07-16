# Optical-flow calibration

This firmware supports three PlatformIO environments:

| Environment | Purpose | Upload command |
| --- | --- | --- |
| `esp32-s3-devkitm-1` | Production application | `pio run -e esp32-s3-devkitm-1 -t upload` |
| `optical_debug` | Human-readable PMW3610 diagnostics | `pio run -e optical_debug -t upload` |
| `calibration` | Machine-readable raw-delta stream for the laptop helper | `pio run -e calibration -t upload` |

The calibration build has no fitting, persistence, or configuration command
path on the ESP32. It only sends raw sensor readings; the laptop performs the
fit and prints JSON to paste into `data/calibration.json`.
After printing the JSON, it also gives a human-readable rotation/scale summary
for each sensor and the X/Y fit residuals.

## Normal driver operation

The PMW3610 driver reads two sensors sharing the SDIO/SCLK bus. Initialize the
bus through `dual_pmw3610_init()` once, then call `dual_pmw3610_poll()` once
per control cycle. The driver resets/configures both sensors during
initialization and alternates which sensor is read first on each poll to avoid
a fixed timing bias.

```cpp
#include "config/pin_map.h"
#include "drivers/pmw3610_driver.h"

static DualPmw3610 sensors;

void setup() {
    const PmwPinConfig pins = {
        .sdio_pin = PIN_PMW_SDIO,
        .sclk_pin = PIN_PMW_SCLK,
        .ncs_l_pin = PIN_PMW_NCS_L,
        .ncs_r_pin = PIN_PMW_NCS_R,
    };
    dual_pmw3610_init(&sensors, &pins);
}

void loop() {
    int16_t ldx, ldy, rdx, rdy;
    bool l_valid, r_valid;
    dual_pmw3610_poll(&sensors, &ldx, &ldy, &l_valid, &rdx, &rdy, &r_valid);

    // Consume only samples whose matching *_valid flag is true.
}
```

### Poll result format

`dual_pmw3610_poll()` does not create a serial packet; it fills six output
arguments with the following per-cycle raw data:

| Field | Type | Meaning |
| --- | --- | --- |
| `ldx`, `ldy` | `int16_t` | Signed raw count delta from the left sensor. |
| `rdx`, `rdy` | `int16_t` | Signed raw count delta from the right sensor. |
| `l_valid`, `r_valid` | `bool` | Whether that sensor's delta can be trusted this cycle. |

Counts are incremental movement since the previous sensor read, not an
integrated position. Treat an invalid reading as unavailable: do not add it to
a pose estimate or substitute it for a real zero. A valid flag requires no
counter overflow, valid laser power, no laser fault, and SQUAL at or above the
configured threshold.

For sensor bring-up or surface diagnostics, use
`pmw3610_bus_burst_read_diagnostics(ncs_pin)`. It returns a
`Pmw3610Diagnostics` structure containing the same `dx`/`dy` data plus motion
status, SQUAL, shutter, and pixel min/average/max values. This full burst is
for diagnostics rather than the timing-sensitive normal polling loop.

### Cumulative odometry UART packet format

To convert raw counts into body-frame motion, configure `Pmw3610Fusion`, pass
each poll result to `pmw3610_fusion_process()`, and integrate valid deltas with
`Pmw3610PoseManager`. The shared `odometry_packet_send()` function serializes
the cumulative pose as a `PACKET_TYPE_ODOMETRY` payload.

```text
offset  size  field
0       4     cumulative x position (float, mm, little-endian)
4       4     cumulative y position (float, mm, little-endian)
8       4     cumulative heading (float, radians, little-endian)
12      4     packet sequence (uint32, little-endian)
16      1     valid (0 or 1)
```

The generic UART frame supplies the `0xAA 0x55` magic, protocol version,
message type, payload length, and XOR checksum. `valid` is `1` only when both
sensor readings are valid. Invalid cycles retain the previous cumulative pose.
`odometry_packet_decode()` performs the inverse operation on a received frame.

### Generic UART link format

Every message sent by `uart_link_send()` has this wire format:

```text
byte(s)  field
0-1      magic: 0xAA, 0x55
2        protocol version: 0x01
3        PacketMessageType
4        payload length: 0-64
5...     payload bytes
last     XOR(version, message type, length, every payload byte)
```

The UART layer treats the payload as opaque bytes. Packet-specific modules are
responsible for encoding and decoding those bytes. To send an arbitrary packet:

```cpp
static UartLink uart_link = {0};

void setup() {
    esp_err_t error = uart_link_init(&uart_link, &DRIVETRAIN_UART_LINK_CONFIG);
    if (error != ESP_OK) {
        // UART is unavailable; handle the initialization failure.
    }
}

void send_status() {
    const uint8_t status_payload[] = {1U, 2U, 3U};
    esp_err_t error = uart_link_send(
        &uart_link,
        PACKET_TYPE_STATUS,
        status_payload,
        (uint8_t)sizeof(status_payload)
    );
    if (error != ESP_OK) {
        // The packet was not fully queued or transmitted.
    }
}
```

Call `uart_link_update()` regularly to drain the hardware RX buffer and feed
the frame parser. A complete valid frame becomes available through
`uart_link_take_packet()`:

```cpp
void receive_packets() {
    if (uart_link_update(&uart_link) != ESP_OK) {
        return;
    }

    PacketFrame packet;
    if (uart_link_take_packet(&uart_link, &packet) != ESP_OK) {
        return;
    }

    switch ((PacketMessageType)packet.message_type) {
        case PACKET_TYPE_ODOMETRY: {
            OdometryPacket odometry;
            if (odometry_packet_decode(&packet, &odometry) == ESP_OK) {
                // Consume cumulative pose and inspect odometry.valid.
            }
            break;
        }

        case PACKET_TYPE_COMMAND:
            // Decode with the command packet module.
            break;

        case PACKET_TYPE_STATUS:
            // Decode with the status packet module.
            break;

        default:
            break;
    }
}
```

The link stores only the latest complete packet. If another packet arrives
before the current one is taken, `packets_overwritten` increments. The other
runtime counters are `packets_sent`, `packets_received`, `checksum_errors`, and
`parse_errors`.

### Complete fused-data example

This is the normal application shape when the static calibration loader and
UART link are enabled. It sends one odometry packet per poll cycle; invalid
sensor cycles are still sent with `valid=0` so the receiver can discard them
without mistaking them for zero motion.

```cpp
#include <Arduino.h>

#include <robot_common/odometry_packet.h>
#include <robot_common/uart_link.h>
#include "config/fusion_config.h"
#include "config/pin_map.h"
#include "config/static_calibration.h"
#include "config/uart_link_config.h"
#include "drivers/pmw3610_driver.h"
#include "sensing/pmw3610_fusion.h"
#include "sensing/pmw3610_pose.h"

static DualPmw3610 sensors;
static Pmw3610Fusion fusion;
static Pmw3610PoseManager pose;
static UartLink uart_link = {0};
static bool fusion_ready = false;
static uint32_t sequence = 0;

void setup() {
    const PmwPinConfig pins = {
        .sdio_pin = PIN_PMW_SDIO,
        .sclk_pin = PIN_PMW_SCLK,
        .ncs_l_pin = PIN_PMW_NCS_L,
        .ncs_r_pin = PIN_PMW_NCS_R,
    };
    dual_pmw3610_init(&sensors, &pins);
    pmw3610_pose_init(&pose);

    FusionConfig config;
    fusion_ready = static_calibration_load(&config);
    if (!fusion_ready) {
        // Do not transmit uncalibrated motion.
        return;
    }
    if (!pmw3610_fusion_configure(&fusion, &config)) {
        fusion_ready = false;
        return;
    }
    if (uart_link_init(&uart_link, &DRIVETRAIN_UART_LINK_CONFIG) != ESP_OK) {
        fusion_ready = false;
    }
}

void loop() {
    if (!fusion_ready) {
        delay(1000);
        return;
    }

    int16_t ldx, ldy, rdx, rdy;
    bool l_valid, r_valid;
    dual_pmw3610_poll(&sensors, &ldx, &ldy, &l_valid, &rdx, &rdy, &r_valid);

    const DeltaPose delta = pmw3610_fusion_process(&fusion, ldx, ldy, rdx, rdy);
    const bool valid = l_valid && r_valid;
    pmw3610_pose_update(&pose, &delta, l_valid, r_valid);

    const OdometryPacket packet = {
        .x_mm = pose.x_mm,
        .y_mm = pose.y_mm,
        .theta_rad = pose.theta_rad,
        .sequence = sequence++,
        .valid = valid,
    };
    odometry_packet_send(&uart_link, &packet);
    uart_link_update(&uart_link);
    delay(10);
}
```

For a bench check without writing an application, flash `optical_debug` and
also upload the filesystem containing `data/calibration.json`:

```sh
pio run -e optical_debug -t upload
pio run -e optical_debug -t uploadfs
```

In addition to the `[L]` and `[R]` raw diagnostic lines, it prints:

```text
[FUSED] d=(forward,lateral,heading_delta_deg) valid=0-or-1 pose=(x,y,heading_deg)
```

`pose` is a bench-only cumulative estimate. It automatically resets to zero
when either sensor is invalid, so use only continuous valid stretches when
checking a forward, lateral, or in-place rotation test.

To verify a calibration, reset the rig at a known origin and make short,
separate motions. A forward move should predominantly change `d` and `pose`'s
first value, a lateral move should predominantly change their second values,
and an in-place turn should predominantly change heading. If forward is
consistently reversed, apply the 180-degree matrix flip described below.

## Calibration procedure

1. Connect the ESP32 to the laptop by USB and close any serial monitor that
   has the port open.
2. Flash the calibration build:

   ```sh
   pio run -e calibration -t upload
   ```

3. Install the one laptop dependency:

   ```sh
   python -m pip install -r tools/requirements-calibration.txt
   ```

4. Start the helper, replacing `COM5` with the ESP32's serial port. On Linux,
   use a path such as `/dev/ttyACM0`.

   ```sh
   python tools/calibrate_optical.py --port COM5
   ```

   Optional arguments:

   ```text
   --baud 115200                 Serial rate (default: 115200)
   --passes 3                    Passes per axis; otherwise prompted
   ```

5. Enter the number of passes per axis. Three or more passes is recommended.
6. For every prompted **X** pass, press Enter to start capture, move the rig
   smoothly along the X/forward axis in either direction, then press Enter to
   stop. Pass directions may be mixed.
7. Repeat for every **Y** pass, again moving in either direction along the
   Y/lateral axis. The helper aligns reversed passes automatically.
8. Copy the printed `left` and `right` objects into
   `data/calibration.json`, preserving the measured `baseline_mm` value.

Example result:

```json
{
  "left": {
    "m00": 0.999500,
    "m01": -0.031619,
    "m10": 0.031619,
    "m11": 0.999500
  },
  "right": {
    "m00": 0.999200,
    "m01": -0.039989,
    "m10": 0.039989,
    "m11": 0.999200
  }
}
```

When the production firmware is configured to load the static JSON file,
upload the filesystem after updating it:

```sh
pio run -e esp32-s3-devkitm-1 -t uploadfs
```

## Calibration serial protocol

The `calibration` firmware emits two newline-terminated ASCII records every
poll cycle (nominally every 10 ms):

```text
CAL,<sensor>,<dx>,<dy>,<valid>\n
```

| Field | Type | Meaning |
| --- | --- | --- |
| `sensor` | `L` or `R` | Left or right PMW3610 sensor. |
| `dx` | signed integer | Raw X count delta for this poll cycle. |
| `dy` | signed integer | Raw Y count delta for this poll cycle. |
| `valid` | `0` or `1` | `1` only when the driver accepts the sample; the helper ignores `0`. |

Example:

```text
CAL,L,12,-3,1
CAL,R,11,-4,1
CAL,L,0,0,0
CAL,R,0,0,0
```

The helper accumulates valid `dx`/`dy` values separately for each sensor and
each pass. It normalizes and averages the X and Y pass directions, then fits
their closest single rotation angle. Scale is fixed at `1.0`: the helper does
not measure or compensate scale, shear, or independent per-axis scaling.

The first X pass defines the positive-forward sign convention. From unlabeled
axis motion alone, a rotation and that same rotation plus 180 degrees are
physically indistinguishable. Use the fused debug check above to confirm that
a known forward movement produces positive forward output. If it is reversed,
multiply all four matrix entries by `-1` (equivalently, add 180 degrees to the
reported rotation).

The generated matrix follows this convention:

```text
[dx_rotated]   [m00 m01] [forward_input]
[dy_rotated] = [m10 m11] [lateral_input]
```

The emitted matrix is always the unit rotation `R(theta)`, so it has the
constrained form `[m00 m01; m10 m11] = [cos(theta) -sin(theta); sin(theta)
cos(theta)]`. In particular, `m00 = m11`, `m01 = -m10`, and its scale is
always `1.0`. The helper rejects X/Y readings that are not sufficiently close
to that rotation-only model.

Because calibration scale is fixed, the JSON only changes direction. The
static configuration loader applies the fixed count/mm conversion derived from
the configured PMW3610 CPI before fusion, so fused and pose output remain in
millimetres. The calibration helper does not adjust that conversion.

Each matrix must be invertible. The helper stops with an error if a pass has no
valid samples, too little movement, or points substantially away from the
other passes for that axis.

## JSON file format

`data/calibration.json` uses this complete schema:

```json
{
  "baseline_mm": 190.5,
  "left":  { "m00": 0.0, "m01": 0.0, "m10": 0.0, "m11": 0.0 },
  "right": { "m00": 0.0, "m01": 0.0, "m10": 0.0, "m11": 0.0 }
}
```

`baseline_mm` is the physical center-to-center separation between the two
sensors, measured with calipers. Every matrix value must be finite, and each
matrix determinant (`m00*m11 - m01*m10`) must be nonzero.

The JSON matrices must be unit rotations. At load time they are multiplied by
the configured PMW3610 count/mm value, producing the count-space matrix used
by fusion; do not put the CPI scale into this file yourself.
