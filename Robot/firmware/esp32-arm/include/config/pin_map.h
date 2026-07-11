#pragma once

// PMW3610 dual-sensor bus (shared 3-wire SPI -- SDIO is bidirectional,
// bit-banged, not a hardware SPI peripheral; NCS_L/NCS_R distinguish the
// two sensors on the shared SDIO/SCLK lines). Defaults below match the
// PMW3610 sensor-node project's confirmed wiring (see that project's
// CLAUDE.md pinout table) -- reconfirm against esp32-arm's actual harness
// before relying on them, since this is different physical hardware.
#define PIN_PMW_SDIO            48  // Shared bus -- both sensors tied together
#define PIN_PMW_SCLK            47  // Shared bus -- both sensors tied together
#define PIN_PMW_NCS_L           38  // Left sensor chip select
#define PIN_PMW_NCS_R           39  // Right sensor chip select

// UART link to esp32-drivetrain (the main control board -- owns
// motors/encoders and does the actual encoder/optical slip-detection
// fusion; this board only streams DELTA,... to it). Placeholder pins --
// esp32-drivetrain's own PIN_TOP_ESP32_UART_RX/TX (GPIO 47/48 on that
// board) are a different chip's pins, not these; confirm actual wiring
// once this board's harness is built.
#define PIN_DRIVETRAIN_UART_TX  17
#define PIN_DRIVETRAIN_UART_RX  18
