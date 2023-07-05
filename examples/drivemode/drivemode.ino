#include <Arduino.h>
#include <Wire.h>
#include "PCAL9535A.h"

PCAL9535A::PCAL9535A<TwoWire> gpio(Wire);

constexpr int port0testpin = 0;
constexpr int port1testpin = 8;

void setup() {
  gpio.begin();
  gpio.portSetOutputMode(PCAL9535A::Port::P0, PCAL9535A::DriveMode::OPENDRAIN);
  gpio.portSetOutputMode(PCAL9535A::Port::P1, PCAL9535A::DriveMode::PUSHPULL);
  gpio.pinMode(port0testpin, OUTPUT);
  gpio.pinMode(port1testpin, OUTPUT);
}

void loop() {
  gpio.digitalWrite(port0testpin, HIGH);
  gpio.digitalWrite(port1testpin, HIGH);
  delay(100);
  gpio.digitalWrite(port0testpin, LOW);
  gpio.digitalWrite(port1testpin, LOW);
  delay(100);
}