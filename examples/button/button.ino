#include <Wire.h>
#include "PCAL9535A.h"

PCAL9535A gpio;
  
void setup() {  
  gpio.begin();      // use default address 0

  gpio.pinMode(0, INPUT);
  gpio.pinSetPull(0, PULL_UP);  // turn on the internal pullup

  pinMode(13, OUTPUT);  // use the p13 LED
}


void loop() {
  digitalWrite(13, gpio.digitalRead(0));
}