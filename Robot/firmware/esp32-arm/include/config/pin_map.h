/* Defines the ESP32 arm board's GPIO assignments for motors, sensors, and links. */
#pragma once

// Servos
#define PIN_SERVO_HABITAT_LEFT_PWM      1
#define PIN_SERVO_HABITAT_RIGHT_PWM     2
#define PIN_SERVO_TOWER_ROTATE_PWM      3
#define PIN_SERVO_TOWER_LEFT_PWM        4
#define PIN_SERVO_TOWER_MIDDLE_PWM      5
#define PIN_SERVO_TOWER_RIGHT_PWM       6
#define PIN_SERVO_ROCK_LIFT_PWM         7
#define PIN_SERVO_ROCK_CLAW_PWM         8

// Solar Panel Microswitch
// TODO: unassigned -- pick a free GPIO and confirm against the actual wiring
// before use.
#define PIN_SOLAR_PANEL_MICROSWITCH GPIO_NUM_NC  // Normally closed to ground; pressed reads HIGH

// Locating Motor
#define PIN_LOC_EN          13      // Power on the locating motor
#define PIN_LOC_SWITCH      14      // Normally closed to ground; pressed reads HIGH

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

// Robot start switch
#define PIN_START_SWITCH     46      // Active low; released state is pulled high

// Time of Flight (I2C)
#define PIN_I2C_SDA         GPIO_NUM_NC
#define PIN_I2C_SCL         GPIO_NUM_NC
// GPIO45/46 were reassigned to the metal detector and start switch. Both
// outer sensors remain disconnected until their XSHUT lines are assigned
// new GPIOs.
#define PIN_TOF_LEFT_XSHUT  GPIO_NUM_NC
#define PIN_TOF_RIGHT_XSHUT GPIO_NUM_NC

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

// Metal Detector
#define PIN_METAL_DETECTOR      45  // Pulse input from metal detector oscillator
