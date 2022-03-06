#include <Arduino.h>
#include <Wire.h>
#include "PCAL9535A.h"

PCAL9535A::PCAL9535A<TwoWire> gpio(Wire);
  
void setup() {  
  gpio.begin();      // use default address 0

  gpio.pinMode(0, INPUT);
  gpio.pinSetPull(0, PCAL9535A::PullSetting::UP);  // turn on the internal pullup

  pinMode(13, OUTPUT);  // use the p13 LED
}


void loop() {
  digitalWrite(13, gpio.digitalRead(0));
}