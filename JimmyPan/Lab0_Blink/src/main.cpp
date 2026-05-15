#include <Arduino.h>

#define LED_PIN 4

// put function declarations here:
void blinkAfterMiliseconds(int milliseconds, int ledPin);

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  blinkAfterMiliseconds(1000, LED_PIN);
}

// put function definitions here:
void blinkAfterMiliseconds(int milliseconds, int ledPin) {
  digitalWrite(ledPin, HIGH);
  delay(milliseconds);
  digitalWrite(ledPin, LOW);
  delay(milliseconds);
  return;
}