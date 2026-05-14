// This code will "blink" pin 4 of the esp32 pico for one second every second.
// connecting a series resistor (such as a 330 ohm) as well as an LED from this pin to ground will blink the LED.
#include <Arduino.h>


#define LED_PIN 4


void setup() {
  pinMode(LED_PIN, OUTPUT); // Sets LED_PIN (in this case defined as pin 4 above) as an output pin
}


void loop() {
  digitalWrite(LED_PIN, HIGH); // Writes the LED_PIN (pin 4 in this case) HIGH (3.3 volts to the pin which in this case will be powering an LED)
  delay(1000); // delays by 1000 milliseconds
  digitalWrite(LED_PIN, LOW); // Writes the LED_PIN (pin 4 in this case) LOW (0 volts)
  delay(1000);
}
