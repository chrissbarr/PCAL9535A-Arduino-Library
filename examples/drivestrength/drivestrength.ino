#include <Arduino.h>
#include <Wire.h>
#include "PCAL9535A.h"

PCAL9535A::PCAL9535A gpio;

constexpr int testpin = 1;
  
void setup() {  
  gpio.begin();      // use default address 0
  gpio.pinMode(testpin, OUTPUT);
}

void loop() {
  gpio.pinSetDriveStrength(testpin, PCAL9535A::DriveStrength::P25);
  gpio.digitalWrite(testpin, HIGH);
  delay(25);
  gpio.digitalWrite(testpin, LOW);
  delay(100);

  gpio.pinSetDriveStrength(testpin, PCAL9535A::DriveStrength::P50);
  gpio.digitalWrite(testpin, HIGH);
  delay(25);
  gpio.digitalWrite(testpin, LOW);
  delay(100);

  gpio.pinSetDriveStrength(testpin, PCAL9535A::DriveStrength::P75);
  gpio.digitalWrite(testpin, HIGH);
  delay(25);
  gpio.digitalWrite(testpin, LOW);
  delay(100);

  gpio.pinSetDriveStrength(testpin, PCAL9535A::DriveStrength::P100);
  gpio.digitalWrite(testpin, HIGH);
  delay(25);
  gpio.digitalWrite(testpin, LOW);
  
  delay(1000);
}