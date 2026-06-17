#include <Arduino.h>
#include "driver/ledc.h"

#define pwmChannel1 0
#define pwmOutpin1 5

#define pwmChannel2 1
#define pwmOutpin2 18

volatile bool buttonPressed = false;


void IRAM_ATTR ButtonPressed() {
  buttonPressed = !buttonPressed;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  ledcSetup(pwmChannel1, 500, 8); // channel, frequency, resolution
  ledcAttachPin(pwmOutpin1, pwmChannel1); // pin, channel

  ledcSetup(pwmChannel2, 500, 8); // channel, frequency, resolution
  ledcAttachPin(pwmOutpin2, pwmChannel2); // pin, channel

  pinMode(4, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(4), ButtonPressed, FALLING);
}

void loop() {

  if (buttonPressed) {
    ledcWrite(pwmChannel2, 0); // duty cycle (0-255 for 8-bit resolution)
    delay(100);
    ledcWrite(pwmChannel1, 128); // duty cycle (0-255 for 8-bit resolution)
  } else {
    ledcWrite(pwmChannel1, 0); // duty cycle (0-255 for 8-bit resolution)
    delay(100);
    ledcWrite(pwmChannel2, 128); // duty cycle (0-255 for 8-bit resolution)
  }

  // Serial.println(buttonPressed);
  // delay(1000);

  // ledcWrite(pwmChannel1, 128); // duty cycle (0-255 for 8-bit resolution)
  // delay(5000);
  // ledcWrite(pwmChannel1, 0); // duty cycle (0-255 for 8-bit resolution)
  // delay(10);

  // ledcWrite(pwmChannel2, 128); // duty cycle (0-255 for 8-bit resolution)
  // delay(5000);
  // ledcWrite(pwmChannel2, 0); // duty cycle (0-255 for 8-bit resolution)
  // delay(10);
}
