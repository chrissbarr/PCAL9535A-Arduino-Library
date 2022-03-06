#include <Arduino.h>
#include <AceWire.h>
#include "PCAL9535A.h"
using ace_wire::SimpleWireInterface;

constexpr int PIN_SDA = 1;
constexpr int PIN_SCL = 2;
constexpr uint8_t DELAY_MICROS = 1;

// Create the AceWire SimpleWireInterface
SimpleWireInterface wire(PIN_SDA, PIN_SCL, DELAY_MICROS);

// Create PCAL9535A instance with reference to SimpleWireInterface
PCAL9535A::PCAL9535A<SimpleWireInterface> gpio(wire);

  
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