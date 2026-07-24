/* Runs the arm board's sensors, odometry UART stream, and Tower-Z demo actions. */
#include <Arduino.h>

#include <robot_common/command_packet.h>
#include <robot_common/status_packet.h>
#include <robot_common/uart_link.h>

#include "comm/odometry_link_producer.h"
#include "config/pin_map.h"
#include "config/stepper_config.h"
#include "config/tof_config.h"
#include "config/uart_link_config.h"
#include "control/time_of_flight/tof_manager.h"
#include "drivers/stepper_driver.h"

namespace {

constexpr uint32_t kStatusRepeatPeriodMs = 20;
constexpr uint32_t kStatusRepeatDurationMs = 500;
constexpr float kTowerTravelMm = 100.0f;

TofManager tof_manager = {};
UartLink drivetrain_uart = {};
OdometryLinkProducer odometry_producer = {};
StepperDriver tower_z_stepper = {};

bool tower_action_active = false;
bool tof_ready = false;
ActionStatusDetail active_action_detail = STATUS_DETAIL_NONE;
ActionStatusDetail repeated_action_detail = STATUS_DETAIL_NONE;
uint32_t repeat_status_until_ms = 0;
uint32_t last_status_ms = 0;

bool deadline_reached(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) >= 0;
}

void send_status(StatusCode code, uint8_t detail) {
    const StatusPacket status = {
        .code = code,
        .detail = detail,
    };
    (void)status_packet_send(&drivetrain_uart, &status);
}

void start_tower_move(CommandOpcode command) {
    if (tower_action_active || stepper_is_moving(&tower_z_stepper)) {
        send_status(STATUS_FAULT, static_cast<uint8_t>(command));
        return;
    }

    const bool moving_up = command == CMD_TOWER_Z_UP;
    const float distance_mm = moving_up ? -kTowerTravelMm : kTowerTravelMm;
    active_action_detail = moving_up
        ? STATUS_DETAIL_TOWER_Z_RAISED
        : STATUS_DETAIL_TOWER_Z_LOWERED;
    repeated_action_detail = STATUS_DETAIL_NONE;
    repeat_status_until_ms = 0;
    stepper_move_distanceMM(&tower_z_stepper, distance_mm);
    tower_action_active = true;
    Serial.printf("# Tower Z moving %s %.0f mm\n",
                  moving_up ? "up" : "down", kTowerTravelMm);
}

void service_commands() {
    if (uart_link_update(&drivetrain_uart) != ESP_OK) return;

    PacketFrame frame = {};
    if (uart_link_take_packet(&drivetrain_uart, &frame) != ESP_OK ||
        !command_packet_is(&frame)) {
        return;
    }

    CommandPacket command = {};
    if (command_packet_decode(&frame, &command) != ESP_OK) return;
    if (command.opcode == CMD_TOWER_Z_UP ||
        command.opcode == CMD_TOWER_Z_DOWN) {
        start_tower_move(command.opcode);
    }
}

bool service_action_status(uint32_t now_ms) {
    if (tower_action_active && !stepper_is_moving(&tower_z_stepper)) {
        tower_action_active = false;
        repeated_action_detail = active_action_detail;
        active_action_detail = STATUS_DETAIL_NONE;
        repeat_status_until_ms = now_ms + kStatusRepeatDurationMs;
        last_status_ms = now_ms - kStatusRepeatPeriodMs;
        Serial.println(repeated_action_detail == STATUS_DETAIL_TOWER_Z_RAISED
                           ? "# Tower Z raised"
                           : "# Tower Z lowered");
    }

    if (repeated_action_detail == STATUS_DETAIL_NONE ||
        deadline_reached(now_ms, repeat_status_until_ms)) {
        repeated_action_detail = STATUS_DETAIL_NONE;
        return false;
    }

    if (now_ms - last_status_ms >= kStatusRepeatPeriodMs) {
        last_status_ms = now_ms;
        send_status(STATUS_ACTION_COMPLETE,
                    static_cast<uint8_t>(repeated_action_detail));
    }
    return true;
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("# Starting arm demo firmware");

    // Bring up the command path before optional sensing. A missing ToF sensor
    // must not prevent the arm from receiving and executing Tower commands.
    ESP_ERROR_CHECK(uart_link_init(
        &drivetrain_uart, &DRIVETRAIN_UART_LINK_CONFIG));
    ESP_ERROR_CHECK(stepper_init(&tower_z_stepper, towerZConfig));
    Serial.println("# Arm UART and Tower-Z stepper ready");

    esp_err_t tof_error = tof_manager_init(&tof_manager, &ARM_TOF_CONFIG);
    if (tof_error == ESP_OK) {
        tof_error = tof_manager_start(&tof_manager);
    }
    tof_ready = (tof_error == ESP_OK);
    if (!tof_ready) {
        Serial.printf("# ToF unavailable (%s); arm commands remain enabled\n",
                      esp_err_to_name(tof_error));
    }

    const PmwPinConfig pmw_pins = {
        .sdio_pin = PIN_PMW_SDIO,
        .sclk_pin = PIN_PMW_SCLK,
        .ncs_l_pin = PIN_PMW_NCS_L,
        .ncs_r_pin = PIN_PMW_NCS_R,
    };
    if (!odometry_link_producer_init(&odometry_producer, &pmw_pins)) {
        Serial.println("# Optical odometry unavailable; arm actions still work");
    }
}

void loop() {
    service_commands();
    stepper_update(&tower_z_stepper);

    if (tof_ready) {
        const esp_err_t tof_error = tof_manager_poll(&tof_manager);
        if (tof_error != ESP_OK && tof_error != ESP_ERR_NOT_FINISHED) {
            tof_ready = false;
            Serial.printf("# ToF polling stopped (%s); arm commands remain enabled\n",
                          esp_err_to_name(tof_error));
        }
    }

    const uint32_t now_ms = millis();
    const bool reporting_completion = service_action_status(now_ms);
    if (!reporting_completion) {
        (void)odometry_link_producer_update(
            &odometry_producer, &drivetrain_uart);
    }

    delay(1);
}
