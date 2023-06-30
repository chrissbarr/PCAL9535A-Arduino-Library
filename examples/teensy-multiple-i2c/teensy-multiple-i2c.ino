#include <Arduino.h>
#include "i2c_driver_wire.h"
#include "PCAL9535A.h"

/**
 * This example uses the multiple I2C interfaces on the Teensy 4.1 board.
 * Uses the teensy4_i2c library here:
 * https://github.com/Richard-Gemmell/teensy4_i2c 
 * 
 * (I2C peripherals are exposed as Wire, Wire1, and Wire2)
 */

PCAL9535A::PCAL9535A<TwoWire> gpio1(Wire);
PCAL9535A::PCAL9535A<TwoWire> gpio2(Wire1);
PCAL9535A::PCAL9535A<TwoWire> gpio3(Wire2);

void printAllGPIO(PCAL9535A::PCAL9535A<TwoWire>& gpio) {
  int state = gpio.readGPIO16();  
  for (int i = 0; i < 16; i++) {
    bool b = state & (1 << 15);
    Serial.print(b);
    state = state << 1;
  }
  Serial.println();
}
  
void setup() {  
  gpio1.begin(PCAL9535A::HardwareAddress::A000);  // 0x20 - Pins = 000
  gpio2.begin(PCAL9535A::HardwareAddress::A000);
  gpio3.begin(PCAL9535A::HardwareAddress::A000);
  Serial.begin(9600);
}

void loop() {
  Serial.println("");
  Serial.print("GPIO1: "); printAllGPIO(gpio1);
  Serial.print("GPIO2: "); printAllGPIO(gpio2);
  Serial.print("GPIO3: "); printAllGPIO(gpio3);
  delay(100);
}