/* Runs arm sensing and the drivetrain-coordinated arm action sequence. */
#include <Arduino.h>

#include <robot_common/app_log.h>
#include <robot_common/fixed_rate_gate.h>
#include <robot_common/uart_link.h>

#include "comm/odometry_link_producer.h"
#include "config/metal_detector_config.h"
#include "config/pin_map.h"
#include "config/stepper_config.h"
// #include "config/tof_config.h"  // ToF disabled until XSHUT wiring is restored.
#include "config/uart_link_config.h"
// #include "control/time_of_flight/tof_manager.h"
#include "control/task/arm_action_dispatcher.h"
#include "control/task/habitat_action_controller.h"
#include "control/task/metal_detector_action_controller.h"
#include "control/task/pi_action_controller.h"
#include "control/task/tower_action_controller.h"
#include "drivers/metal_detector_driver.h"
#include "drivers/stepper_driver.h"

namespace {

// TofManager tof_manager = {};
UartLink drivetrain_uart = {};
UartLink pi_uart = {};
OdometryLinkProducer odometry_producer = {};
StepperDriver tower_x_stepper = {};
StepperDriver tower_z_stepper = {};
StepperDriver habitat_x_stepper = {};
StepperDriver habitat_z_stepper = {};
TowerActionController tower_action_controller = {};
HabitatActionController habitat_action_controller = {};
PiActionController pi_action_controller = {};
MetalDetectorDriver metal_detector = {};
MetalDetectorActionController metal_detector_action_controller = {};
ArmActionDispatcher arm_action_dispatcher = {};
// bool tof_ready = false;

constexpr int64_t kOdometrySendPeriodUs = 5000;  // 200 Hz, matches the drivetrain's control loop
FixedRateGate odometry_gate = {kOdometrySendPeriodUs, 0};

constexpr uint32_t kLocatorSwitchReportPeriodMs = 100;  // 10 Hz for switch diagnostics
uint32_t last_locator_switch_report_ms = 0;

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);
    app_log_init();
    Serial.println("# Starting arm firmware");

    // Bring up the command path before optional sensing. A missing ToF sensor
    // must not prevent the arm from receiving and executing arm commands.

    // UART init
    ESP_ERROR_CHECK(uart_link_init(
        &drivetrain_uart, &DRIVETRAIN_UART_LINK_CONFIG));
    ESP_ERROR_CHECK(uart_link_init(
        &pi_uart, &PI_UART_LINK_CONFIG));
    Serial.println("# Drivetrain and Pi UARTs ready");

    // Locator init
    pinMode(PIN_LOC_EN, OUTPUT);
    digitalWrite(PIN_LOC_EN, LOW);
    Serial.println("# Locator retracted");

    // Tower init
    tower_action_controller_init(
        &tower_action_controller,
        &drivetrain_uart,
        &tower_x_stepper,
        &tower_z_stepper);
    Serial.println(
        "# Tower X/Z startup positions are the manually adjusted home");
    Serial.println("# Tower servos and steppers ready");

    // Habitat init
    ESP_ERROR_CHECK(stepper_init(&habitat_x_stepper, habitatXConfig));
    ESP_ERROR_CHECK(stepper_init(&habitat_z_stepper, habitatZConfig));

    habitat_action_controller_init(
        &habitat_action_controller,
        &drivetrain_uart,
        &habitat_x_stepper,
        &habitat_z_stepper);
    Serial.println(
        "# Habitat X/Z startup positions are the manually adjusted home");
    Serial.println("# Habitat servos and steppers ready");

    pi_action_controller_init(
        &pi_action_controller,
        &pi_uart,
        &drivetrain_uart);
    Serial.println("# PI action controller initialized");

    // Metal detector init
    esp_err_t metal_detector_error =
        metal_detector_driver_init(&metal_detector, &METAL_DETECTOR_CONFIG);
    if (metal_detector_error == ESP_OK) {
        metal_detector_error = metal_detector_driver_start(&metal_detector);
    }
    if (metal_detector_error == ESP_OK) {
        // Capture one complete, live no-metal reference for early reads.
        metal_detector_error =
            metal_detector_driver_begin_sample(&metal_detector);
    }
    if (metal_detector_error == ESP_OK) {
        MetalDetectorSample startup_sample = {};
        do {
            delay(1);
            metal_detector_error = metal_detector_driver_poll_sample(
                &metal_detector, &startup_sample);
        } while (metal_detector_error == ESP_ERR_NOT_FINISHED);

        if (metal_detector_error == ESP_OK) {
            metal_detector_error = metal_detector_driver_set_baseline(
                &metal_detector, &startup_sample);
        }
    }
    if (metal_detector_error != ESP_OK) {
        if (metal_detector_driver_is_enabled(&metal_detector)) {
            (void)metal_detector_driver_stop(&metal_detector);
        }
        Serial.printf(
            "# Metal detector unavailable (%s); arm commands remain enabled\n",
            esp_err_to_name(metal_detector_error));
    }
    metal_detector_action_controller_init(
        &metal_detector_action_controller,
        &drivetrain_uart,
        &metal_detector);
    Serial.println("# Metal detector action controller initialized");

    arm_action_dispatcher_init(
        &arm_action_dispatcher,
        &drivetrain_uart,
        &tower_action_controller,
        &habitat_action_controller,
        &pi_action_controller,
        &metal_detector_action_controller);
    Serial.println("# Arm action dispatcher initialized");

    // ToF disabled until the XSHUT wiring is restored.
    // esp_err_t tof_error = tof_manager_init(&tof_manager, &ARM_TOF_CONFIG);
    // if (tof_error == ESP_OK) {
    //     tof_error = tof_manager_start(&tof_manager);
    // }
    // tof_ready = (tof_error == ESP_OK);
    // if (!tof_ready) {
    //     Serial.printf("# ToF unavailable (%s); arm commands remain enabled\n",
    //                   esp_err_to_name(tof_error));
    // }

    // Optical init
    const PmwPinConfig pmw_pins = {
        .sdio_pin = PIN_PMW_SDIO,
        .sclk_pin = PIN_PMW_SCLK,
        .ncs_l_pin = PIN_PMW_NCS_L,
        .ncs_r_pin = PIN_PMW_NCS_R,
    };
    if (!odometry_link_producer_init(&odometry_producer, &pmw_pins)) {
        Serial.println("# Optical odometry unavailable; arm actions still work");
    }

    // The drivetrain starts its sequence when this ESP sends its ready status.
    pinMode(PIN_START_SWITCH, INPUT_PULLUP);
    Serial.println("# Waiting for start switch");
    while (digitalRead(PIN_START_SWITCH) != LOW) {
        delay(1);
    }
    Serial.println("# Start switch pressed");
    delay(1000);  // Allow one second before starting the robot sequence.
}

void loop() {
    stepper_update(&tower_x_stepper);
    stepper_update(&tower_z_stepper);
    stepper_update(&habitat_x_stepper);
    stepper_update(&habitat_z_stepper);

    // ToF disabled until the XSHUT wiring is restored.
    // if (tof_ready) {
    //     const esp_err_t tof_error = tof_manager_poll(&tof_manager);
    //     if (tof_error != ESP_OK && tof_error != ESP_ERR_NOT_FINISHED) {
    //         tof_ready = false;
    //         Serial.printf("# ToF polling stopped (%s); arm commands remain enabled\n",
    //                       esp_err_to_name(tof_error));
    //     }
    // }

    const uint32_t now_ms = millis();
    Serial.println(digitalRead(PIN_LOC_SWITCH));
    const bool reporting_arm_status = arm_action_dispatcher_update(
        &arm_action_dispatcher, now_ms);

    int64_t unused_dt_us = 0;
    const int64_t now_us = static_cast<int64_t>(now_ms) * 1000;
    if (!reporting_arm_status && odometry_gate.Ready(now_us, &unused_dt_us)) {
        (void)odometry_link_producer_update(
            &odometry_producer, &drivetrain_uart);
    }

    delay(1);
}
