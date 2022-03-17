# PCAL9535A Arduino Library
This is an Arduino library for the PCAL9535A I2C GPIO Expander IC. It should be compatible with:
- PCA9535
- PCA9535A
- PCAL9535A

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
