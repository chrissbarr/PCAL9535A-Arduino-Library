#include <Arduino.h>
#include <SoftwareWire.h>
#include "PCAL9535A.h"

constexpr int PIN_SDA = 1;
constexpr int PIN_SCL = 2;

SoftwareWire softwareWire(PIN_SDA, PIN_SCL);

PCAL9535A::PCAL9535A<SoftwareWire> gpio(softwareWire);

void setup() {
  gpio.begin();      // use default address 0
  gpio.pinMode(0, OUTPUT);
}

void loop() {
  delay(100);
  gpio.digitalWrite(0, HIGH);
  delay(100);
  gpio.digitalWrite(0, LOW);
}