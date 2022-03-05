#include <Arduino.h>
#include <Wire.h>
#include "PCAL9535A.h"

PCAL9535A::PCAL9535A gpio;
  
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