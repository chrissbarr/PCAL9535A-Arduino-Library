#include <Arduino.h>
#include <Wire.h>
#include "PCAL9535A.h"

PCAL9535A::PCAL9535A<TwoWire> gpio1(Wire);
PCAL9535A::PCAL9535A<TwoWire> gpio2(Wire);
PCAL9535A::PCAL9535A<TwoWire> gpio3(Wire);
PCAL9535A::PCAL9535A<TwoWire> gpio4(Wire);
  
void setup() {  
  gpio1.begin(PCAL9535A::HardwareAddress::A000);  // 0x20 - Pins = 000
  gpio2.begin(PCAL9535A::HardwareAddress::A001);  // 0x21 - Pins = 001
  gpio3.begin(PCAL9535A::HardwareAddress::A010);  // 0x22 - Pins = 010
  gpio4.begin(PCAL9535A::HardwareAddress::A011);  // 0x23 - Pins = 011

  gpio1.pinMode(0, INPUT);
  gpio2.pinMode(0, INPUT);
  gpio3.pinMode(0, OUTPUT);
  gpio4.pinMode(0, OUTPUT);
}

void loop() {
  gpio1.digitalWrite(0, gpio3.digitalRead(0));
  gpio2.digitalWrite(0, gpio4.digitalRead(0));
  delay(100);
}