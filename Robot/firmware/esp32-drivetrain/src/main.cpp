/* Starts and services the ordered robot task sequence. */
#include <Arduino.h>

#include <robot_common/uart_link.h>

#include "config/communication/uart_link_config.h"
#include "config/drivetrain/drivetrain_config.h"
#include "config/tape_following/tape_following_config.h"
#include "control/drivetrain/drivetrain.h"
#include "control/odometry/pose_service.h"
#include "control/odometry/pose_tracker.h"
#include "control/task/robot_sequence_controller.h"
#include "drivers/tape_sensor/tape_sensor_driver.h"

namespace {

UartLink arm_uart = {};
RobotSequenceController robot_sequence_controller = {};

// Runs continuously from startup, independent of the (still-placeholder)
// movement/task controllers above, so any future lego block can read a
// live pose without wiring its own hardware access.
Drivetrain drivetrain = {};
TapeSensorMux tape_sensor_mux = {};
TapeSensor tape_sensors[TAPE_SENSOR_MODULE_COUNT] = {};
TapeSensor *tape_sensor_list[TAPE_SENSOR_MODULE_COUNT] = {
    &tape_sensors[0], &tape_sensors[1], &tape_sensors[2]};
DrivetrainOdometrySourceConfig pose_encoder_config = {};
PoseTracker pose_tracker = {};
Pmw3610OdometryLink arm_odometry_link = {};
PoseService pose_service = {};

// Brings up the drivetrain, tape sensors, and pose tracker so pose is live
// from startup for whatever lego block needs it later.
esp_err_t initialize_pose_tracking() {
    esp_err_t err = drivetrain_init(&drivetrain, &DRIVETRAIN_CONFIG);
    if (err == ESP_OK) err = drivetrain_enable(&drivetrain);
    if (err == ESP_OK) err = tape_sensor_mux_init(&tape_sensor_mux, &TAPE_SENSOR_MUX_CONFIG);
    if (err == ESP_OK) {
        err = tape_sensor_driver_init(&tape_sensors[0], &FRONT_TAPE_SENSOR_CONFIG, &tape_sensor_mux);
    }
    if (err == ESP_OK) {
        err = tape_sensor_driver_init(&tape_sensors[1], &BACK_TAPE_SENSOR_CONFIG, &tape_sensor_mux);
    }
    if (err == ESP_OK) {
        err = tape_sensor_driver_init(&tape_sensors[2], &LEFT_TAPE_SENSOR_CONFIG, &tape_sensor_mux);
    }
    if (err != ESP_OK) return err;

    pose_encoder_config.x_drive_kinematics = DRIVETRAIN_CONFIG.x_drive_kinematics;
    pose_encoder_config.counts_per_revolution_fl =
        DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_FL]->counts_per_revolution;
    pose_encoder_config.counts_per_revolution_fr =
        DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_FR]->counts_per_revolution;
    pose_encoder_config.counts_per_revolution_bl =
        DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_BL]->counts_per_revolution;
    pose_encoder_config.counts_per_revolution_br =
        DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_BR]->counts_per_revolution;
    return pose_tracker_init(&pose_tracker, &pose_encoder_config);
}

}  // namespace

void setup() {
    // Must be called first, prevents weird motor behavior at start up
    drivetrain_hold_safe_outputs(&DRIVETRAIN_CONFIG);

    Serial.begin(115200);
    Serial.println("Starting...");

    // Delay(5000) later need to be change to start button action
    delay(5000);

    Serial.println("# Starting Task Sequence");

    // UART init
    esp_err_t err = uart_link_init(&arm_uart, &TOP_ESP_UART_LINK_CONFIG);
    if (err != ESP_OK) {
        Serial.printf("# FAULT: arm UART initialization failed (%s)\n", esp_err_to_name(err));
        return;
    }

    // Robot task sequence init
    err = robot_sequence_controller_init(&robot_sequence_controller, &arm_uart);
    if (err != ESP_OK) {
        Serial.printf(
            "# FAULT: Robot sequence initialization failed (%s)\n",
            esp_err_to_name(err));
        return;
    }

    err = initialize_pose_tracking();
    if (err != ESP_OK) {
        Serial.printf("# FAULT: pose tracking init failed (%s)\n", esp_err_to_name(err));
    }

    pose_service = {
        .pose_tracker = &pose_tracker,
        .drivetrain = &drivetrain,
        .arm_uart = &arm_uart,
        .odometry_link = &arm_odometry_link,
        .sequence_controller = &robot_sequence_controller,
    };
}

void loop() {
    const uint32_t now_ms = millis();

    // Runs every tick regardless of sequence/tape state, so pose and
    // arm_uart never stall behind either one.
    pose_service_update(&pose_service, now_ms);
    robot_sequence_controller_update(&robot_sequence_controller, now_ms);

    // Feed data to drivetrain watchdog to sample encoders when not stationary
    drivetrain_set_body_velocity(&drivetrain, 0.0f, 0.0f, 0.0f);
    const int64_t now_us = static_cast<int64_t>(now_ms) * 1000;
    drivetrain_update(&drivetrain, now_us);

    (void)tape_sensor_driver_read_all(tape_sensor_list);

    delay(1);
}
