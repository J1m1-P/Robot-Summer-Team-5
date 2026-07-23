/* Declares the PMW3610-to-drivetrain odometry packet producer. */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <robot_common/odometry_packet.h>
#include <robot_common/uart_link.h>

#include "config/pmw3610_config.h"
#include "drivers/pmw3610_driver.h"
#include "sensing/pmw3610_fusion.h"
#include "sensing/pmw3610_pose.h"

#ifdef __cplusplus
extern "C" {
#endif

// Owns the dual-PMW3610 sensors, fusion, and cumulative pose used to feed
// the drivetrain board's odometry link. This board only forwards fused
// pose samples -- fault policy and sensor-fusion-with-encoders decisions
// belong to the drivetrain board's odometry module, not here.
typedef struct {
    DualPmw3610 dual;
    Pmw3610Fusion fusion;
    Pmw3610PoseManager pose;
    uint32_t sequence;
    bool ready;
    // Per-sensor hardware validity from the most recent poll (see
    // pmw3610_status_valid()) -- not needed for production, but lets a
    // bench harness tell "one sensor is dead/misaligned" apart from "both
    // sensors are simply off the tracking surface" without a second,
    // register-clearing poll of its own.
    bool last_l_valid;
    bool last_r_valid;
} OdometryLinkProducer;

// Loads the static PMW3610 calibration, configures fusion, and initializes
// both sensors. Returns false (leaving the producer inert for this boot)
// when calibration is missing or invalid -- the drivetrain board simply
// never receives odometry packets and falls back to its other sources.
bool odometry_link_producer_init(OdometryLinkProducer *producer,
                                 const PmwPinConfig *pins);

// Polls both sensors, fuses and integrates one cycle of motion, and sends
// the resulting cumulative pose as an odometry packet over `drivetrain_link`.
// A no-op (returns ESP_OK) when the producer failed to initialize.
esp_err_t odometry_link_producer_update(OdometryLinkProducer *producer,
                                        UartLink *drivetrain_link);

#ifdef __cplusplus
}
#endif
