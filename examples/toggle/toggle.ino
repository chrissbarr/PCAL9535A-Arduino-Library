#include <Wire.h>
#include "PCAL9535A.h"

PCAL9535A gpio;
  
void setup() {  
  gpio.begin();
  gpio.pinMode(0, OUTPUT);
}

void loop() {
  delay(100);

  gpio.digitalWrite(0, HIGH);

  delay(100);

  gpio.digitalWrite(0, LOW);
}