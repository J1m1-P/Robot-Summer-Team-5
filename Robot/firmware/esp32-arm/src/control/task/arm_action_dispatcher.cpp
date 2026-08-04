/* Implements shared UART routing for Tower, Habitat, and Pi actions. */
#include "control/task/arm_action_dispatcher.h"

#include <Arduino.h>

#include <robot_common/command_packet.h>
#include <robot_common/status_packet.h>

#include "config/pin_map.h"

namespace {

constexpr uint32_t kReadyRepeatPeriodMs = 20;
constexpr uint32_t kSolarPanelContactRepeatPeriodMs = 20;

// Helper function to send Status Packet to drivetrain through UART
void send_status(
    ArmActionDispatcher *dispatcher,
    StatusCode code,
    uint8_t detail) {

    const StatusPacket status = {
        .code = code,
        .detail = detail,
    };
    (void)status_packet_send(dispatcher->drivetrain_uart, &status);
}

// The normally-closed switch grounds the pin while open. When pressed, the
// circuit opens and INPUT_PULLUP makes the pin read HIGH. Repeat the contact
// status while pressed so the drivetrain cannot miss it at an action boundary.
bool report_solar_panel_contact(
    ArmActionDispatcher *dispatcher,
    uint32_t now_ms) {
    if (digitalRead(PIN_SOLAR_PANEL_MICROSWITCH) != HIGH ||
        now_ms - dispatcher->last_solar_panel_contact_status_ms <
            kSolarPanelContactRepeatPeriodMs) {
        return false;
    }

    dispatcher->last_solar_panel_contact_status_ms = now_ms;
    send_status(
        dispatcher,
        STATUS_SOLAR_PANEL_CONTACT,
        STATUS_DETAIL_NONE);
    return true;
}

// Receives one drivetrain command and routes it to Tower, Habitat, or RPI.
void service_command(ArmActionDispatcher *dispatcher, uint32_t now_ms) {

    // Read available drivetrain UART data into the packet parser.
    const esp_err_t update_error =
        uart_link_update(dispatcher->drivetrain_uart);
    if (update_error != ESP_OK) {
        if (!dispatcher->uart_fault_logged) {
            dispatcher->uart_fault_logged = true;
            Serial.printf(
                "# Arm UART update failed (%s)\n",
                esp_err_to_name(update_error));
        }
        return;
    }
    dispatcher->uart_fault_logged = false;

    // Take the oldest queued packet.
    PacketFrame frame = {};
    if (uart_link_take_packet(
            dispatcher->drivetrain_uart, &frame) != ESP_OK) {
        return;
    }

    // Decode the packet as an arm command.
    CommandPacket command = {};
    const esp_err_t decode_error =
        command_packet_decode(&frame, &command);
    if (decode_error != ESP_OK) {
        Serial.printf(
            "# Arm command decode failed (%s)\n",
            esp_err_to_name(decode_error));
        return;
    }

    // Identify the controller that owns this command.
    const bool is_tower =
        tower_action_controller_accepts(command.opcode);
    const bool is_habitat =
        habitat_action_controller_accepts(command.opcode);
    const bool is_pi =
        pi_action_controller_accepts(command.opcode);
    const bool supported = is_tower || is_habitat || is_pi;

    // Keep remote and physical arm actions sequential.
    const bool arm_busy =
        tower_action_controller_is_busy(dispatcher->tower_controller) ||
        habitat_action_controller_is_busy(dispatcher->habitat_controller) ||
        pi_action_controller_is_busy(dispatcher->pi_controller);

    // Reject unsupported commands or commands received while busy.
    if (!supported || arm_busy) {
        const char *reason =
            supported ? "another arm action is busy" : "unsupported action";
        Serial.printf(
            "# Arm command rejected: %s (opcode %u)\n",
            reason,
            static_cast<unsigned>(command.opcode));
        send_status(
            dispatcher,
            STATUS_FAULT,
            static_cast<uint8_t>(command.opcode));
        return;
    }

    // Stop advertising startup readiness after accepting the first command.
    dispatcher->readiness_pending = false;
    // The drivetrain only sends a Pi command once it already knows the Pi is
    // ready (that's what unblocked it), so the beacon has done its job.
    if (is_pi) dispatcher->pi_ready_announce_pending = false;

    // Start the action on its owning controller.
    if (is_tower) {
        tower_action_controller_start(
            dispatcher->tower_controller, &command);
    } else if (is_habitat) {
        habitat_action_controller_start(
            dispatcher->habitat_controller, &command);
    } else {
        pi_action_controller_start(
            dispatcher->pi_controller, &command, now_ms);
    }
}

}  // namespace

// Called during setup(). Initializes the dispatcher.
void arm_action_dispatcher_init(
    ArmActionDispatcher *dispatcher,
    UartLink *drivetrain_uart,
    TowerActionController *tower_controller,
    HabitatActionController *habitat_controller,
    PiActionController *pi_controller) {

    *dispatcher = {};
    dispatcher->drivetrain_uart = drivetrain_uart;
    dispatcher->tower_controller = tower_controller;
    dispatcher->habitat_controller = habitat_controller;
    dispatcher->pi_controller = pi_controller;
    dispatcher->readiness_pending = true;
    pinMode(PIN_SOLAR_PANEL_MICROSWITCH, INPUT_PULLUP);
}

// Called during loop() to service UART, readiness, and both controllers.
bool arm_action_dispatcher_update(
    ArmActionDispatcher *dispatcher,
    uint32_t now_ms) {

    service_command(dispatcher, now_ms);

    if (dispatcher->readiness_pending) {
        // Only reserve the shared UART on the specific tick a status packet
        // is actually sent, not for the whole (potentially indefinite, until
        // the drivetrain issues its first arm command) readiness-pending
        // window -- otherwise odometry sending in main.cpp's loop() stays
        // blocked the entire time, since it also checks this return value.
        const bool sending_this_tick = now_ms - dispatcher->last_ready_status_ms >=
                                        kReadyRepeatPeriodMs;
        if (sending_this_tick) {
            dispatcher->last_ready_status_ms = now_ms;
            send_status(
                dispatcher,
                STATUS_ACTION_COMPLETE,
                STATUS_DETAIL_NONE);
        }
        return sending_this_tick;
    }

    // Begin (and keep repeating, for the same lost-packet-tolerance reason as
    // the startup beacon above) a STATUS_PI_READY beacon once the Pi's own
    // ready packet has been seen, so the drivetrain can gate the teletubby
    // search on it without talking to the Pi directly.
    if (!dispatcher->pi_ready_announce_pending &&
        pi_action_controller_is_ready(dispatcher->pi_controller)) {
        dispatcher->pi_ready_announce_pending = true;
    }
    bool reporting_pi_ready = false;
    if (dispatcher->pi_ready_announce_pending) {
        reporting_pi_ready = now_ms - dispatcher->last_pi_ready_status_ms >=
                              kReadyRepeatPeriodMs;
        if (reporting_pi_ready) {
            dispatcher->last_pi_ready_status_ms = now_ms;
            send_status(dispatcher, STATUS_PI_READY, STATUS_DETAIL_NONE);
        }
    }

    // Update Tower
    const bool reporting_tower = tower_action_controller_update(
        dispatcher->tower_controller, now_ms);

    // Update Habitat
    const bool reporting_habitat = habitat_action_controller_update(
        dispatcher->habitat_controller, now_ms);

    // Update the Pi request/report transaction.
    const bool reporting_pi = pi_action_controller_update(
        dispatcher->pi_controller, now_ms);

    const bool reporting_solar_panel = report_solar_panel_contact(
        dispatcher, now_ms);

    // Reserve the shared UART while any controller reports completion.
    return reporting_tower || reporting_habitat || reporting_pi || reporting_pi_ready || reporting_solar_panel;
}
