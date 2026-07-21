#include <Arduino.h>

#include "config/drivetrain/drivetrain_config.h"
#include "control/drivetrain/drivetrain.h"
#include "control/drivetrain/wheel_velocity_controller.h"
#include "drivers/encoder/encoder_driver.h"
#include "drivers/motor/motor_driver.h"

// -----------------------------------------------------------------------------
// Real-time tuning only
// This entire file is an isolated RAM-only wheel-controller tuning application.
// -----------------------------------------------------------------------------

// Multi-wheel PI tuning harness -- see platformio.ini's [env:tuning]. Not
// the production entry point; kept entirely separate from main.cpp.
//
// Any subset of the 4 motor/encoder pairs can be enabled simultaneously
// (see the "motor" command below) -- each enabled one gets its own
// MotorDriver/EncoderDriver/WheelVelocityController instance and runs the control
// loop independently, but all enabled motors share one
// WheelVelocityControllerConfig (ff/ffo/kp/ki) and one step/continuous schedule,
// so multiple wheels driven with the same gains can be compared side by
// side in one run.
//
// Self-driving step-response test (or continuous mode -- see
// "continuous" below): alternates the shared target speed between 0 and
// kStepTargetMps every kStepPeriodMs, runs each enabled wheel's PI loop
// every iteration, and streams one CSV line per enabled motor at
// kPrintPeriodMs (50 Hz, not every control iteration -- printing at the
// full control-loop rate would flood the terminal and leave no room to
// type commands) -- "millis,motor,target_mps,measured_mps,duty" (motor is
// 1..4) -- plottable with tools/tuning_dashboard.html.
//
// Serial commands (newline-terminated), typeable any time between steps:
//   ff <value>       -- set feedforward slope gain (kff), shared by all enabled motors
//   ffo <value>      -- set feedforward offset (kff_offset), shared
//   kp <value>       -- set proportional gain, shared
//   ki <value>       -- set integral gain, shared
//   target <value>   -- set the step's "high" target speed, m/s, shared
//   period <value>   -- set the step period, ms, shared
//   invert <0|1>     -- flip the sign of the measured speed reading, shared
//   continuous <0|1> -- 0 = step mode (default): target alternates
//                     0/step_target_mps every step_period_ms, for step-
//                     response tuning. 1 = continuous mode: target holds
//                     at step_target_mps constantly (no alternating) --
//                     for watching steady-state behavior/drift over a
//                     longer run instead of a repeated step. "target" and
//                     "period" still apply in continuous mode: target
//                     sets the held speed, period is ignored while
//                     continuous but remembered for when you switch back.
//   motor <1..4> <0|1> -- enable/disable one motor/encoder pair:
//                     1=M1/E1 (FL), 2=M2/E2 (BL), 3=M3/E3 (FR), 4=M4/E4 (BR)
//                     -- physical connector order, see motor_config.c's
//                     per-motor comments. Enabling initializes that pair and resets its PI
//                     integral; disabling stops it. Independent of the
//                     other three -- any combination can be on at once.
//
// invert is a tuning-session convenience for when the encoder reports the
// opposite sign from what the commanded duty direction would suggest --
// it only flips the value fed into this loop and printed over serial, it
// does NOT touch EncoderDriverConfig.direction_inverted (that's
// compile-time, in config/drivetrain/encoder_config.c, and is the real fix once you
// know which way it should go).
//
// All of the above live only in RAM here -- they reset to this file's
// compiled-in defaults on reboot. Once you land on values you like, copy
// the gains into config/drivetrain/drivetrain_config.c by hand; this build never
// writes anything persistent.
//
// To characterize kff specifically: enable one motor at a time, set kp=0
// and ki=0 (pure open loop), then try a few different ff values and read
// the steady-state measured_mps for the current target step -- kff is
// roughly (duty that produced the target speed) / target_mps.
//
// Also usable headlessly from tools/tuning_dashboard.html (Web Serial API
// browser page) for live charting and point-and-click gain entry instead
// of typing raw commands -- same protocol, same commands, that page just
// sends these same lines and parses the same CSV output.

namespace {

constexpr int kMotorCount = 4;

constexpr float kDefaultStepTargetMps = 1.0f;
constexpr unsigned long kDefaultStepPeriodMs = 3000;

// The PI control loop itself runs every iteration of loop() (as fast as
// the ESP32 can go, likely thousands of Hz) -- that's fine, more control
// updates only helps. But printing a CSV line every single iteration
// floods the terminal and leaves no room to type commands, so telemetry
// is rate-limited separately at kPrintPeriodMs. 50 Hz is still <10% of
// the 115200 baud link's throughput at this line length (~30 bytes/line
// -> ~1.5 KB/s of ~11.5 KB/s available) even with a single motor enabled;
// with all 4 enabled the per-tick output is proportionally larger
// (~4x), still comfortably under the link's capacity.
constexpr unsigned long kPrintPeriodMs = 20; // 50 Hz

const MotorDriverConfig *const kMotorConfigs[kMotorCount] = {
    DRIVETRAIN_CONFIG.motor_configs[DRIVETRAIN_MOTOR_FL],
    DRIVETRAIN_CONFIG.motor_configs[DRIVETRAIN_MOTOR_BL],
    DRIVETRAIN_CONFIG.motor_configs[DRIVETRAIN_MOTOR_FR],
    DRIVETRAIN_CONFIG.motor_configs[DRIVETRAIN_MOTOR_BR],
};
const EncoderDriverConfig *const kEncoderConfigs[kMotorCount] = {
    DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_FL],
    DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_BL],
    DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_FR],
    DRIVETRAIN_CONFIG.encoder_configs[DRIVETRAIN_MOTOR_BR],
};
const char *const kMotorNames[kMotorCount] = {
    "M1/E1 (FL)", "M2/E2 (BL)", "M3/E3 (FR)", "M4/E4 (BR)"
};

MotorDriver motors[kMotorCount] = {};
EncoderDriver encoders[kMotorCount];
WheelVelocityController controller_states[kMotorCount];
bool motor_enabled[kMotorCount] = {true, false, false, false}; // M1 enabled by default, matching the old single-motor default

WheelVelocityControllerConfig controller_config = DRIVETRAIN_CONFIG.wheel_controller;

float step_target_mps = kDefaultStepTargetMps;
unsigned long step_period_ms = kDefaultStepPeriodMs;
bool invert_measurement = false;
bool continuous_mode = false;

unsigned long last_update_ms = 0;
unsigned long last_print_ms = 0;
unsigned long step_start_ms = 0;
bool step_high = false;

void print_usage() {
    Serial.println("# usage: ff <value> | ffo <value> | kp <value> | ki <value> | target <value> | period <value> | invert <0|1> | continuous <0|1> | motor <1..4> <0|1>");
}

// Enables or disables one motor/encoder pair independently of the other
// three. Enabling initializes that pair's hardware and resets its PI
// integral (old accumulator state doesn't mean anything for a fresh
// start); disabling stops it. Safe to call redundantly (enabling an
// already-enabled motor, or disabling an already-disabled one) --
// motor_driver_disable()/encoder_driver_stop() guard on !initialized and
// the *_init()/_start()/_enable() calls are idempotent re-configurations.
void set_motor_enabled(int index, bool enabled) {
    if (index < 0 || index >= kMotorCount) {
        Serial.println("# motor index must be 1..4");
        return;
    }

    if (enabled) {
        encoder_driver_init(&encoders[index], kEncoderConfigs[index]);
        encoder_driver_start(&encoders[index]);

        motor_driver_init(&motors[index], kMotorConfigs[index]);
        motor_driver_enable(&motors[index]);

        wheel_velocity_controller_reset(&controller_states[index]);
    } else {
        motor_driver_disable(&motors[index]);
        encoder_driver_stop(&encoders[index]);
    }

    motor_enabled[index] = enabled;

    Serial.print("# motor ");
    Serial.print(kMotorNames[index]);
    Serial.println(enabled ? " = enabled" : " = disabled");
}

void handle_serial_command() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    const int firstSpace = line.indexOf(' ');
    if (firstSpace < 0) {
        print_usage();
        return;
    }

    const String key = line.substring(0, firstSpace);
    String rest = line.substring(firstSpace + 1);
    rest.trim();

    if (key == "motor") {
        const int secondSpace = rest.indexOf(' ');
        if (secondSpace < 0) {
            Serial.println("# usage: motor <1..4> <0|1>");
            return;
        }
        const int index = rest.substring(0, secondSpace).toInt() - 1;
        const float enabledValue = rest.substring(secondSpace + 1).toFloat();
        set_motor_enabled(index, enabledValue != 0.0f);
        return;
    }

    const float value = rest.toFloat();

    if (key == "ff") {
        controller_config.kff = value;
        Serial.print("# kff = ");
        Serial.println(controller_config.kff, 4);
    } else if (key == "ffo") {
        controller_config.kff_offset = value;
        Serial.print("# kff_offset = ");
        Serial.println(controller_config.kff_offset, 4);
    } else if (key == "kp") {
        controller_config.kp = value;
        Serial.print("# kp = ");
        Serial.println(controller_config.kp, 4);
    } else if (key == "ki") {
        controller_config.ki = value;
        Serial.print("# ki = ");
        Serial.println(controller_config.ki, 4);
    } else if (key == "target") {
        step_target_mps = value;
        Serial.print("# target_mps = ");
        Serial.println(step_target_mps, 4);
    } else if (key == "period") {
        if (value > 0.0f) {
            step_period_ms = (unsigned long)value;
            step_start_ms = millis(); // restart the step cycle cleanly on the new period
            Serial.print("# period_ms = ");
            Serial.println(step_period_ms);
        } else {
            Serial.println("# period must be > 0");
        }
    } else if (key == "invert") {
        invert_measurement = (value != 0.0f);
        Serial.print("# invert = ");
        Serial.println(invert_measurement ? "1" : "0");
    } else if (key == "continuous") {
        continuous_mode = (value != 0.0f);
        step_start_ms = millis(); // clean restart of the step cycle if/when we return to step mode
        step_high = false;
        Serial.print("# continuous = ");
        Serial.println(continuous_mode ? "1" : "0");
    } else {
        print_usage();
    }
}

}  // namespace

// Initializes serial telemetry and the default front-left tuning channel.
void setup() {
    Serial.begin(115200);
    delay(1000);

    set_motor_enabled(0, true); // default to M1/E1 (FL) only

    Serial.println("millis,motor,target_mps,measured_mps,duty");

    const unsigned long now = millis();
    last_update_ms = now;
    step_start_ms = now;
}

// Services commands and advances every enabled wheel's tuning controller.
void loop() {
    handle_serial_command();

    const unsigned long now = millis();
    const float dt_s = (now - last_update_ms) / 1000.0f;
    if (dt_s <= 0.0f) return;
    last_update_ms = now;

    float target_mps;
    if (continuous_mode) {
        target_mps = step_target_mps;
    } else {
        if (now - step_start_ms >= step_period_ms) {
            step_start_ms = now;
            step_high = !step_high;
        }
        target_mps = step_high ? step_target_mps : 0.0f;
    }

    const bool should_print = (now - last_print_ms >= kPrintPeriodMs);
    if (should_print) last_print_ms = now;

    for (int i = 0; i < kMotorCount; i++) {
        if (!motor_enabled[i]) continue;

        encoder_driver_update(&encoders[i]);
        float measured_mps = encoder_driver_get_velocity_mps(&encoders[i]);
        if (invert_measurement) measured_mps = -measured_mps;

        float duty = 0.0f;
        wheel_velocity_controller_update(&controller_states[i], &controller_config, target_mps, measured_mps, dt_s, &duty);
        motor_driver_set_duty(&motors[i], duty);

        if (should_print) {
            Serial.print(now);
            Serial.print(',');
            Serial.print(i + 1);
            Serial.print(',');
            Serial.print(target_mps, 4);
            Serial.print(',');
            Serial.print(measured_mps, 4);
            Serial.print(',');
            Serial.println(duty, 4);
        }
    }
}
