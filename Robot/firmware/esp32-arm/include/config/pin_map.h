/* Defines the ESP32 arm board's GPIO assignments for motors, sensors, and links. */
#pragma once

// Servos
#define PIN_SERVO_HABITAT_LEFT_PWM      1
#define PIN_SERVO_HABITAT_RIGHT_PWM     2
#define PIN_SERVO_TOWER_ROTATE_PWM      3
#define PIN_SERVO_TOWER_LEFT_PWM        4
#define PIN_SERVO_TOWER_MIDDLE_PWM      5
#define PIN_SERVO_TOWER_RIGHT_PWM       6
#define PIN_SERVO_SOLAR_PANEL_PWM       45

// Locating Motor
#define PIN_LOC_EN          13      // Power on the locating motor
#define PIN_LOC_SWITCH      14      // Microswtich on the locating motor

// Stepper Motor Driver
#define PIN_STEP1           16      // PWM (not really) for Stepper Motor1
#define PIN_STEP1_DIR       15      // Direction for Stepper Motor1
#define PIN_STEP2           42      // PWM (not really) for Stepper Motor2
#define PIN_STEP2_DIR       41      // Direction for Stepper Motor2
#define PIN_STEP3           18      // PWM (not really) for Stepper Motor3
#define PIN_STEP3_DIR       17      // Direction for Stepper Motor3
#define PIN_STEP4           21      // PWM (not really) for Stepper Motor4
#define PIN_STEP4_DIR       40      // Direction for Stepper Motor4

// Communication

// Time of Flight (I2C)
#define PIN_I2C_SDA         7
#define PIN_I2C_SCL         8
// Temporary bench wiring. GPIO45/46 are ESP32-S3 strapping pins and will be
// replaced by externally controlled lines in the final hardware.
#define PIN_TOF_LEFT_XSHUT  46
#define PIN_TOF_RIGHT_XSHUT 45

// Optical Sensors (SPI)
#define PIN_PMW_SDIO        9       // Shared bus -- both sensors tied together
#define PIN_PMW_SCLK        10      // Shared bus -- both sensors tied together
#define PIN_PMW_NCS_L       11      // Left sensor chip select
#define PIN_PMW_NCS_R       12      // Right sensor chip select

// UART to Drivetrain
#define PIN_DRIVETRAIN_UART_RX  48
#define PIN_DRIVETRAIN_UART_TX  47

// UART to PI
#define PIN_PI_UART_RX          39
#define PIN_PI_UART_TX          38
