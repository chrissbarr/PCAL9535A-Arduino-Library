# PCAL9535A Arduino Library
This is an Arduino library for the PCAL9535A I2C GPIO Expander IC. It is also compatible with a range of other ICs that share in the common register set (see Compatibility below).

## Compatibility

The register set in the PCAL9535A is common to a range of I2C GPIO expanders from both TI and NXP (and probably others).

The base feature set is described by the PCA9535. The more advanced variants (PCAL) include more configuration of the IO. The library is compatible with both, but note the more basic parts do not support all features.

The PCAL9535A has the most full feature set (that I am aware of) so this library targets that part.

The following table shows several parts known to work with this library. The feature set of these parts is summarised:

| Part | Bits | Addresses | Input | Output | Interrupt | Polarity Inv | Prg. Drive Strength | Latchable Inputs | Pull-Up/Down | Output Mode |
|------|------|-----------|-------|--------|-----------|--------------|---------------------|------------------|--------------|-------------|
| PCA9535 | 16 | 8 (0x20 - 0x27) | Yes | Yes | Yes | Yes | No | No | None | Push-Pull |
| PCA9535A | 16 | 8 (0x20 - 0x27) | Yes | Yes | Yes | Yes | No | No | None | Push-Pull |
| PCA9535C | 16 | 8 (0x20 - 0x27) | Yes | Yes | Yes | Yes | No | No | None | Open-Drain |
| PCAL9535A | 16 | 8 (0x20 - 0x27) | Yes | Yes | Yes (maskable) | Yes | Yes | Yes | Programmable | Open-Drain / Push-Pull |
| PCA6416A | 16 | 2 (0x20 - 0x21) | Yes | Yes | Yes | Yes | No | No | None | Push-Pull |
| PCAL6416A | 16 | 2 (0x20 - 0x21) | Yes | Yes | Yes (maskable) | Yes | Yes | Yes | Programmable | Open-Drain / Push-Pull |

Note that other GPIO expanders not listed may be compatible. If you find a non-listed part that is compatible, let me know!

## Key Features
- 2 x 8-bit ports of GPIO (16 pins total).
- Implements Arduino-style `pinMode()`, `digitalRead()`, `digitalWrite()` functions.
- Up to 8 x PCAL9535A on a single I2C bus via address select pins (128 GPIO on I2C bus).
- Supports hardware I2C via Arduino Wire library, or software I2C via third-party libraries (see examples). 
  - Software I2C allows PCAL9535A to be controlled by any two GPIO pins on microcontroller (therefore allowing > 8 PCAL9535A if desired).
- Per-pin configuration as input or output.
- Per-pin configuration of input pin:
  - Internal pullup/pulldown.
  - Latching behaviour.
  - Signal inversion.
  - Interrupts enabled/disabled.
- Per-pin configuration of output pin:
  - Drive strength.
- Per-port configuration of:
  - Output drive mode (push-pull or open-drain)

## Examples
Refer to the examples folder for more detailed examples on usage. 

### Single Input
This example demonstrates reading a single input pin and printing to the serial port.
```
#include <Arduino.h>
#include <Wire.h>
#include "PCAL9535A.h"

PCAL9535A::PCAL9535A<TwoWire> gpio(Wire);
  
void setup() {  
  gpio.begin();
  gpio.pinMode(0, INPUT);
  Serial.begin(9600);
}

void loop() {
  Serial.println(gpio.digitalRead(0));
  delay(1000);
}
```

### Software I2C
This example demonstrates using software I2C (via the external AceWire library) to control the PCAL9535A. This software-I2C implementation should work on any Arduino platform that implements digitalWrite/digitalRead functions (tested working on Arduino Uno and Arduino Due).
```
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
```
