#include <Arduino.h>
#include "driver/ledc.h" // package for pwm output

#define pwmChannel 0
#define PWM_PIN 18
#define BRK_PIN 7
#define DIR_PIN 4


void setup() {
  pinMode(BRK_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PWM_PIN, OUTPUT);
  ledcSetup(pwmChannel,9000,12); // (pwmchannel to use,  frequency in Hz, number of bits)
  ledcAttachPin(PWM_PIN, pwmChannel); // (pin to attach, pwm channel to use)
}

void loop() {
  digitalWrite(BRK_PIN, HIGH); // set brake pin to low to allow motor to run
  digitalWrite(DIR_PIN, HIGH); // set direction pin to high to set direction of motor
  ledcWrite(pwmChannel, 2000); // writes a dutycycle to the specified pwmchannel (which in this case was linked to pin 4)
  delay(200);
}