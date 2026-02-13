# PCAL9535A Arduino Library

[![PlatformIO CI](https://github.com/chrissbarr/PCAL9535A-Arduino-Library/actions/workflows/main.yml/badge.svg)](https://github.com/chrissbarr/PCAL9535A-Arduino-Library/actions/workflows/main.yml)
[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)
[![PlatformIO Registry](https://badges.registry.platformio.org/packages/chrissb/library/PCAL9535A%20Library.svg)](https://registry.platformio.org/libraries/chrissb/PCAL9535A%20Library)

This is an Arduino library for the PCAL9535A I2C GPIO Expander IC. It is also compatible with a range of other ICs that share the common register set (see [Compatibility](#compatibility)).

## Key Features
- **Full-featured GPIO expansion:** 16 pins with programmable pull-up/down, drive strength, latching, and polarity inversion
- **Arduino-compatible API:** Drop-in replacement syntax with `pinMode()`, `digitalRead()`, `digitalWrite()`
- **Flexible I2C support:** Works with any Wire-compatible library via C++ templates (hardware or software I2C)
- **Scalable:** Up to 8 chips per bus (128 GPIO pins) using address select pins + as many buses as you want
- **Per-pin interrupts:** Maskable interrupts with interrupt source identification
- **Configurable outputs:** Push-pull or open-drain modes with adjustable drive strength (25%, 50%, 75%, 100%)
- **Performance optimised:** Whole-port read/write operations for multi-pin control

## Compatibility

The register set in the PCAL9535A is common to a range of I2C GPIO expanders from both TI and NXP (and probably others).

The base feature set is described by the PCA9535. The more advanced variants (PCAL) include more configuration of the IO, such as programmable drive strength, pull-up/down resistors, and push-pull or open-drain outputs. The library supports the full feature set of the PCAL9535A. Parts with a reduced feature set are supported (but obviously those features won't work).

The following table shows several parts known to work with this library. The feature set of these parts is summarised:

| Part | Bits | Addresses | Input | Output | Interrupt | Polarity Inv | Prg. Drive Strength | Latchable Inputs | Pull-Up/Down | Output Mode | Datasheet |
|------|------|-----------|-------|--------|-----------|--------------|---------------------|------------------|--------------|-------------|-----------|
| PCA9535 | 16 | 8 (0x20 - 0x27) | Yes | Yes | Yes | Yes | No | No | None | Push-Pull | [Datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9535_PCA9535C.pdf) |
| PCA9535A | 16 | 8 (0x20 - 0x27) | Yes | Yes | Yes | Yes | No | No | None | Push-Pull | [Datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9535A.pdf) |
| PCA9535C | 16 | 8 (0x20 - 0x27) | Yes | Yes | Yes | Yes | No | No | None | Open-Drain | [Datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9535_PCA9535C.pdf) |
| PCAL9535A | 16 | 8 (0x20 - 0x27) | Yes | Yes | Yes (maskable) | Yes | Yes | Yes | Programmable | Open-Drain / Push-Pull | [Datasheet](https://www.nxp.com/docs/en/data-sheet/PCAL9535A.pdf) |
| PCA6416A | 16 | 2 (0x20 - 0x21) | Yes | Yes | Yes | Yes | No | No | None | Push-Pull | [Datasheet](https://www.nxp.com/docs/en/data-sheet/PCA6416A.pdf) |
| PCAL6416A | 16 | 2 (0x20 - 0x21) | Yes | Yes | Yes (maskable) | Yes | Yes | Yes | Programmable | Open-Drain / Push-Pull | [Datasheet](https://www.nxp.com/docs/en/data-sheet/PCAL6416A.pdf) |

Note that other GPIO expanders not listed may be compatible. If you find a non-listed part that is compatible, let me know!

## API Reference

### Enums

All enums are in the `PCAL9535A` namespace.

| Enum | Values | Description |
|------|--------|-------------|
| `HardwareAddress` | `A000` - `A111` | I2C address offset set by A0/A1/A2 pins |
| `Port` | `P0`, `P1` | Port selection (P0 = pins 0-7, P1 = pins 8-15) |
| `PullSetting` | `NONE`, `UP`, `DOWN` | Internal pull resistor configuration |
| `DriveStrength` | `P25`, `P50`, `P75`, `P100` | Output drive strength (% of max current) |
| `DriveMode` | `PUSHPULL`, `OPENDRAIN` | Output driver mode |

### Methods

**Initialisation**
- `void begin()` — Initialise with default address (0x20).
- `void begin(HardwareAddress addr)` — Initialise with specified address offset.

**Basic GPIO** (Arduino-style)
- `void pinMode(uint8_t pin, uint8_t mode)` — Set pin as `INPUT` or `OUTPUT`.
- `uint8_t digitalRead(uint8_t pin)` — Read pin state (returns 0 or 1).
- `void digitalWrite(uint8_t pin, uint8_t value)` — Set output pin `HIGH` or `LOW`.

**Port-Level I/O**
- `uint8_t readGPIO(Port port)` — Read all 8 pins of a port.
- `void writeGPIO(Port port, uint8_t value)` — Write all 8 pins of a port.
- `uint16_t readGPIO16()` — Read all 16 pins.
- `void writeGPIO16(uint16_t value)` — Write all 16 pins.

**Input Pin Configuration**
- `void pinSetPull(uint8_t pin, PullSetting pull)` — Set pull-up, pull-down, or none.
- `void pinSetInputInversion(uint8_t pin, bool invert)` — Enable/disable polarity inversion.
- `void pinSetInputLatch(uint8_t pin, bool latch)` — Enable/disable input latching.

**Output Pin Configuration**
- `void pinSetDriveStrength(uint8_t pin, DriveStrength strength)` — Set output drive strength.
- `void portSetOutputMode(Port port, DriveMode mode)` — Set push-pull or open-drain per port.

**Interrupts**
- `void pinSetInterruptEnabled(uint8_t pin, bool enabled)` — Enable/disable interrupt on a pin.
- `uint8_t getLastInterruptPin()` — Get pin number that last triggered an interrupt (255 if none).
- `uint8_t getInterruptPinValue()` — Get value of the pin that triggered the last interrupt.

## Installation

**PlatformIO:**
Add to your `platformio.ini`:
```ini
lib_deps = chrissb/PCAL9535A Library
```

**Manual:**
Clone or download this repository into your Arduino `libraries/` folder.

## Quick Start

Here's a minimal example to get started easily:

```cpp
#include <Wire.h>
#include "PCAL9535A.h"

PCAL9535A::PCAL9535A<TwoWire> gpio(Wire);

void setup() {
  gpio.begin();                  // Initialise with default address 0x20
  gpio.pinMode(0, OUTPUT);       // Set pin 0 as output
}

void loop() {
  gpio.digitalWrite(0, HIGH);    // Turn pin 0 on
  delay(500);
  gpio.digitalWrite(0, LOW);     // Turn pin 0 off
  delay(500);
}
```

## Pin Mapping

The PCAL9535A provides 16 GPIO pins organised into two 8-bit ports:

| Pin Numbers | Port | Physical Pins |
|-------------|------|---------------|
| 0-7         | Port 0 (P0) | IO0_0 to IO0_7 |
| 8-15        | Port 1 (P1) | IO1_0 to IO1_7 |

When calling functions like `pinMode(pin, mode)` or `digitalWrite(pin, value)`, use pin numbers 0-15.

## I2C Address

The PCAL9535A I2C address is configured using three address pins (A0, A1, A2):

| A2 | A1 | A0 | I2C Address (Hex) | I2C Address (Decimal) |
|----|----|----|-------------------|----------------------|
| 0  | 0  | 0  | 0x20              | 32                   |
| 0  | 0  | 1  | 0x21              | 33                   |
| 0  | 1  | 0  | 0x22              | 34                   |
| 0  | 1  | 1  | 0x23              | 35                   |
| 1  | 0  | 0  | 0x24              | 36                   |
| 1  | 0  | 1  | 0x25              | 37                   |
| 1  | 1  | 0  | 0x26              | 38                   |
| 1  | 1  | 1  | 0x27              | 39                   |

**Default address:** 0x20 (all address pins tied to GND)

**Multiple devices:** Up to 8 PCAL9535A chips can share a single I2C bus by configuring different addresses.

## Wiring

Basic connection to an Arduino:

| Arduino Pin | PCAL9535A Pin | Notes |
|-------------|---------------|-------|
| 5V or 3.3V  | VCC          | Supply voltage (check your chip's voltage rating) |
| GND         | GND          | Ground |
| SDA (A4 on Uno) | SDA      | I2C data line (requires 4.7kΩ pull-up resistor to VCC) |
| SCL (A5 on Uno) | SCL      | I2C clock line (requires 4.7kΩ pull-up resistor to VCC) |
| Digital Pin (optional) | INT | Interrupt output (active low) |
| GND or VCC  | A0, A1, A2   | Address select pins (tie to GND or VCC to set I2C address) |

**Note:** Most Arduino boards and I2C devices have built-in pull-up resistors, but external 4.7kΩ resistors may be needed for reliable operation, especially with long wires or multiple devices.

## Advanced Usage

### Using Alternative I2C Implementations

The library uses C++ templates to support different I2C implementations. Pass the I2C interface type as a template parameter:

**Hardware I2C (Arduino Wire):**
```cpp
#include <Wire.h>
#include "PCAL9535A.h"

PCAL9535A::PCAL9535A<TwoWire> gpio(Wire);
```

**Software I2C (AceWire):**
```cpp
#include <AceWire.h>
#include "PCAL9535A.h"

using ace_wire::SimpleWireInterface;
SimpleWireInterface wire(SDA_PIN, SCL_PIN, DELAY_MICROS);
PCAL9535A::PCAL9535A<SimpleWireInterface> gpio(wire);
```

**Software I2C (SoftwareWire):**
```cpp
#include <SoftwareWire.h>
#include "PCAL9535A.h"

SoftwareWire wire(SDA_PIN, SCL_PIN);
PCAL9535A::PCAL9535A<SoftwareWire> gpio(wire);
```

See the [softi2c-acewire](examples/softi2c-acewire) and [softi2c-softwarewire](examples/softi2c-softwarewire) examples for complete code.

### Multiple Devices on One Bus

Use different I2C addresses to control multiple PCAL9535A chips:

```cpp
PCAL9535A::PCAL9535A<TwoWire> gpio1(Wire);
PCAL9535A::PCAL9535A<TwoWire> gpio2(Wire);
PCAL9535A::PCAL9535A<TwoWire> gpio3(Wire);

void setup() {
  gpio1.begin(PCAL9535A::HardwareAddress::A000);  // Address 0x20
  gpio2.begin(PCAL9535A::HardwareAddress::A001);  // Address 0x21
  gpio3.begin(PCAL9535A::HardwareAddress::A010);  // Address 0x22
}
```

See the [multiple](examples/multiple) example for complete code.

### Whole-Port Operations

For better performance when reading or writing multiple pins, use port-level operations:

```cpp
// Read all 8 pins of Port 0 at once
uint8_t port0_state = gpio.readGPIO(PCAL9535A::Port::P0);

// Write all 8 pins of Port 1 at once
gpio.writeGPIO(PCAL9535A::Port::P1, 0xFF);  // All high

// Read all 16 pins at once
uint16_t all_pins = gpio.readGPIO16();

// Write all 16 pins at once
gpio.writeGPIO16(0xAAAA);  // Alternating pattern
```

### Interrupt Handling

Enable interrupts on specific pins and read interrupt status:

```cpp
// Enable interrupt on pin 0
gpio.pinSetInterruptEnabled(0, true);

// In your interrupt service routine or main loop:
uint8_t interrupt_pin = gpio.getLastInterruptPin();
if (interrupt_pin != 255) {  // 255 means no interrupt
  uint8_t pin_value = gpio.getInterruptPinValue();
  // Handle the interrupt
}
```

## Examples

The following example sketches are included:

| Example | Description |
|---------|-------------|
| [button](examples/button) | Read a button with pull-up and control an LED |
| [drivemode](examples/drivemode) | Configure open-drain vs push-pull output mode |
| [drivestrength](examples/drivestrength) | Cycle through drive strength settings (25%-100%) |
| [multiple](examples/multiple) | Use multiple PCAL9535A on one I2C bus |
| [readall](examples/readall) | Read all 16 pins and print to serial |
| [softi2c-acewire](examples/softi2c-acewire) | Software I2C via AceWire library |
| [softi2c-softwarewire](examples/softi2c-softwarewire) | Software I2C via SoftwareWire library |
| [teensy-multiple-i2c](examples/teensy-multiple-i2c) | Multiple I2C buses on Teensy 4.1 |
| [toggle](examples/toggle) | Simple output pin toggling |


## Platform Compatibility

This library has been tested on the following platforms:

| Platform | Architecture | Status |
|----------|-------------|--------|
| Arduino Uno | AVR (ATmega328P) | ✅ Tested |
| Teensy 3.1 | ARM Cortex-M4 | ✅ Tested |
| Arduino Due | ARM Cortex-M3 | ✅ Tested |
| ESP32 | Xtensa LX6 | ✅ Tested |
| Teensy 4.1 | ARM Cortex-M7 | ✅ Tested |

The library should work on any platform that supports the Arduino Wire library or compatible I2C implementations.

## Dependencies

Any I2C/Wire library compatible with the Arduino Wire signature, including:
- Arduino Wire library (included with Arduino IDE & most common Arduino-compatible platforms)
- [AceWire](https://github.com/bxparks/AceWire) (bxparks/AceWire @ 0.4.1) - for software I2C support
- [SoftwareWire](https://github.com/Testato/SoftwareWire) (testato/SoftwareWire @ 1.6.0) - alternative software I2C
- [teensy4_i2c](https://github.com/Richard-Gemmell/teensy4_i2c) - for using multiple I2C buses on Teensy 4.1

## Contributing

Contributions are welcome! Here's how you can help:

1. **Report bugs:** Open an issue on [GitHub Issues](https://github.com/chrissbarr/PCAL9535A-Arduino-Library/issues)
2. **Submit pull requests:**
   - Ensure all CI tests pass
   - Follow the existing code style (uses clang-format)
   - Add examples for new features when applicable
3. **Share compatible hardware:** If you find other compatible GPIO expander chips, let me know!

## License

This library is licensed under [GPL-3.0-or-later](LICENSE.txt).
