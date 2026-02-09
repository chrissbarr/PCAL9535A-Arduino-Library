/***************************************************
 This is a library for the PCAL9535A I2C GPIO expander.

 Written by Chris Barr, 2019.
 ****************************************************/

#ifndef _PCAL9535A_H_
#define _PCAL9535A_H_

#include <stdint.h>

// These macros from the Arduino core are defined here and undefined at the end of the header
// to allow use of these macros within this header without them leaking externally.
#define pca_bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define pca_bitSet(value, bit) ((value) |= (1UL << (bit)))
#define pca_bitClear(value, bit) ((value) &= ~(1UL << (bit)))
#define pca_bitWrite(value, bit, bitvalue) (bitvalue ? pca_bitSet(value, bit) : pca_bitClear(value, bit))

namespace PCAL9535A {

constexpr int PCAL9535A_ADDRESS = 0x20;

enum class HardwareAddress : uint8_t {
  A000 = 0x00,
  A001 = 0x01,
  A010 = 0x02,
  A011 = 0x03,
  A100 = 0x04,
  A101 = 0x05,
  A110 = 0x06,
  A111 = 0x07
};

// registers
enum class RegisterAddress : uint8_t {
  P0_INPUT    = 0x00,
  P0_OUTPUT   = 0x02,
  P0_POLINV   = 0x04,
  P0_CONFIG   = 0x06,
  P0_DRVSTR1  = 0x40,
  P0_DRVSTR2  = 0x41,
  P0_ILATCH   = 0x44,
  P0_PULLENA  = 0x46,
  P0_PULLSEL  = 0x48,
  P0_INTMASK  = 0x4A,
  P0_INTSTAT  = 0x4C,
  P1_INPUT    = 0x01,
  P1_OUTPUT   = 0x03,
  P1_POLINV   = 0x05,
  P1_CONFIG   = 0x07,
  P1_DRVSTR1  = 0x42,
  P1_DRVSTR2  = 0x43,
  P1_ILATCH   = 0x45,
  P1_PULLENA  = 0x47,
  P1_PULLSEL  = 0x49,
  P1_INTMASK  = 0x4B,
  P1_INTSTAT  = 0x4D,
  OUTPUT_CONF = 0x4F
};

enum class DriveStrength : uint8_t  {
  P25 = 0b00,
  P50 = 0b01,
  P75 = 0b10,
  P100 = 0b11
};

enum class RegisterValues_PULLENA : uint8_t  {
  PULL_DISABLED = 0x00,
  PULL_ENABLED = 0x01
};

enum class RegisterValues_PULLSEL : uint8_t  {
  PULL_PULLDOWN = 0x00,
  PULL_PULLUP = 0x01
};

enum class DriveMode : uint8_t  {
  PUSHPULL = 0x00,
  OPENDRAIN = 0x01
};

enum class PullSetting : uint8_t  {
  NONE,
  UP,
  DOWN
};

enum class Port : uint8_t {
  P0 = 0x00,
  P1 = 0x01
};

constexpr int PCAL9535A_INT_ERR = 255;

template <typename WIRE>
class PCAL9535A {
public:

  explicit PCAL9535A(WIRE& wire) : mWire(wire) {}
  /**
   * Initializes the PCAL9535A given its HW address, see datasheet for address selection.
   * \param addr Address of PCAL9535A (0 - 7)
   */
  void begin(HardwareAddress addr)
  {
    _i2caddr = PCAL9535A_ADDRESS | static_cast<uint8_t>(addr);

    mWire.begin();

    // set all pins as inputs
    writeRegister(RegisterAddress::P0_CONFIG, 0xff);
    writeRegister(RegisterAddress::P1_CONFIG, 0xff);
  }

  /**
   * Initializes the PCAL9535A at the default address (0)
   */
  void begin()
  {
    begin(HardwareAddress::A000);
  }

  /**
   * Writes an 8-bit value to the entire port given.
   * \param port Port to write to (0 or 1)
   * \param value Value to write to port
   */
  void writeGPIO(Port port, uint8_t value)
  {
    mWire.beginTransmission(_i2caddr);
    mWire.write(static_cast<uint8_t>(port == Port::P0 ? RegisterAddress::P0_OUTPUT : RegisterAddress::P1_OUTPUT));
    mWire.write(value);
    mWire.endTransmission();
  }

  /**
   * Writes a 16-bit value to both ports.
   * \param value Value to write
   */
  void writeGPIO16(uint16_t value)
  {
    mWire.beginTransmission(_i2caddr);
    mWire.write(static_cast<uint8_t>(RegisterAddress::P0_OUTPUT));
    mWire.write(value & 0xFF);
    mWire.write(value >> 8);
    mWire.endTransmission();
  }

  /**
   * Reads all 16 pins (port 0 and 1) into a single 16-bit variable.
   * \return Variable containing all pin values as bits.
   */
  uint16_t readGPIO16()
  {
    uint16_t val = 0;
    uint8_t p0;

    // read the current GPIO inputs
    mWire.beginTransmission(_i2caddr);
    mWire.write(static_cast<uint8_t>(RegisterAddress::P0_INPUT));
    mWire.endTransmission();

    mWire.requestFrom(_i2caddr, uint8_t(2));
    p0 = mWire.read();
    val = mWire.read();
    val <<= 8;
    val |= p0;

    return val;
  }

  /**
   * Read a single port (0 or 1) and return its current 8 bit value.
   * \param port Port to read (0 or 1)
   * \return 8-bit value of port
   */
  uint8_t readGPIO(Port port)
  {
    mWire.beginTransmission(_i2caddr);
    mWire.write(static_cast<uint8_t>(port == Port::P0 ? RegisterAddress::P0_INPUT : RegisterAddress::P1_INPUT));
    mWire.endTransmission();
    mWire.requestFrom(_i2caddr, uint8_t(1));
    return mWire.read();
  }

  /**
   * Sets the mode of a given pin to either INPUT or OUTPUT
   * \param pin Pin to set (0 to 15)
   * \param mode Mode to set to (INPUT or OUTPUT)
   */
  void pinMode(uint8_t pin, uint8_t mode)
  {
    updateRegisterBit(pin, (mode == 0 ? 1 : 0), RegisterAddress::P0_CONFIG, RegisterAddress::P1_CONFIG);
  }

  /**
   * Sets the given pin (if configured as an output) to be either HIGH or LOW
   * \param pin Pin to set (0 to 15)
   * \param value Value to set to (LOW or HIGH)
   */
  void digitalWrite(uint8_t pin, uint8_t value)
  {
    const RegisterAddress regAddr = pinToReg(pin, RegisterAddress::P0_OUTPUT, RegisterAddress::P1_OUTPUT);

    // read the current GPIO state, so we can modify only this one pin
    uint8_t gpio = readRegister(regAddr);

    // update the GPIO state with the pin we wish to set
    pca_bitWrite(gpio, pinToBit(pin), value);

    // write the new GPIO state back out
    writeRegister(regAddr, gpio);
  }

  /**
   * Reads the given pin (if configured as an input) and returns the value
   * \param pin Pin to read (0 to 15)
   * \return Value of pin (LOW or HIGH)
   */
  uint8_t digitalRead(uint8_t pin)
  {
    const RegisterAddress regAddr = pinToReg(pin, RegisterAddress::P0_INPUT, RegisterAddress::P1_INPUT);
    return (readRegister(regAddr) >> pinToBit(pin)) & 0x1;
  }

  /**
   * Set pull-up/pull-down behaviour of a given pin
   * \param pin Pin to set behaviour for
   * \param pull Pullup/pulldown setting to apply
   */
  void pinSetPull(uint8_t pin, PullSetting pull)
  {
    updateRegisterBit(pin, static_cast<uint8_t>((pull == PullSetting::NONE) ? RegisterValues_PULLENA::PULL_DISABLED : RegisterValues_PULLENA::PULL_ENABLED), RegisterAddress::P0_PULLENA, RegisterAddress::P1_PULLENA);
    updateRegisterBit(pin, static_cast<uint8_t>((pull == PullSetting::UP) ? RegisterValues_PULLSEL::PULL_PULLUP : RegisterValues_PULLSEL::PULL_PULLDOWN), RegisterAddress::P0_PULLSEL, RegisterAddress::P1_PULLSEL);
  }

  /**
   * Set drive strength behaviour of a given pin
   * \param pin Pin to set behaviour for
   * \param strength Drive strength setting to apply
   */
  void pinSetDriveStrength(uint8_t pin, DriveStrength strength)
  {
    RegisterAddress regAddr;
    uint8_t regValue;

    if (pinToBit(pin) < 4) {
      regAddr = pinToReg(pin, RegisterAddress::P0_DRVSTR1, RegisterAddress::P1_DRVSTR1);
    }
    else
    {
      regAddr = pinToReg(pin, RegisterAddress::P0_DRVSTR2, RegisterAddress::P1_DRVSTR2);
    }

    regValue = readRegister(regAddr);
    regValue &= ~(0x03 << ((pin % 4) * 2));
    regValue |= (static_cast<uint8_t>(strength) & 0x03) << ((pin % 4) * 2);

    writeRegister(regAddr, regValue);
  }

  /**
   * Set input-inversion behaviour of a given pin
   * \param pin Pin to set behaviour for
   * \param invert Inversion setting to apply (pin input is inverted if true)
   */
  void pinSetInputInversion(uint8_t pin, bool invert)
  {
    updateRegisterBit(pin, (invert ? 1 : 0), RegisterAddress::P0_POLINV, RegisterAddress::P1_POLINV);
  }

  /**
   * Set input latching behaviour of a given pin
   * \param pin Pin to set behaviour for
   * \param latch Latch setting to apply (pin input is latching if true)
   */
  void pinSetInputLatch(uint8_t pin, bool latch)
  {
    updateRegisterBit(pin, (latch ? 1 : 0), RegisterAddress::P0_ILATCH, RegisterAddress::P1_ILATCH);
  }

  /**
   * Set interrupts enabled/disabled for a given pin
   * \param pin Pin to set behaviour for
   * \param enabled Enable / disable interrupts
   */
  void pinSetInterruptEnabled(uint8_t pin, bool enabled)
  {
    updateRegisterBit(pin, (enabled ? 0 : 1), RegisterAddress::P0_INTMASK, RegisterAddress::P1_INTMASK);
  }

  /**
   * Get the last pin to trigger an interrupt
   * \return Pin which triggered interrupt
   */
  uint8_t getLastInterruptPin()
  {
    uint8_t intf;

    intf = readRegister(RegisterAddress::P0_INTSTAT);
    for(int i = 0; i < 8; i++) {
      if (pca_bitRead(intf, i)) {
        return i;
      }
    }

    intf = readRegister(RegisterAddress::P1_INTSTAT);
    for(int i = 0; i < 8; i++) {
      if (pca_bitRead(intf, i)) {
        return i + 8;
      }
    }

    return PCAL9535A_INT_ERR;
  }

  /**
   * Get the value of the last pin to trigger an interrupt
   * \return Value of pin which triggered interrupt
   */
  uint8_t getInterruptPinValue()
  {
    uint8_t intPin = getLastInterruptPin();

    if(intPin != PCAL9535A_INT_ERR){
      return (digitalRead(intPin));
    }

    return PCAL9535A_INT_ERR;
  }

  /**
   * Configure the port drive mode (either push-pull or open-drain)
   * \param port Port to apply the setting to
   * \param mode Mode to set the port to
   */
  void portSetOutputMode(Port port, DriveMode mode)
  {
    uint8_t regValue = readRegister(RegisterAddress::OUTPUT_CONF);
    pca_bitWrite(regValue, (port == Port::P0 ? 0 : 1), (static_cast<uint8_t>(mode) & 0x01));
    writeRegister(RegisterAddress::OUTPUT_CONF, regValue);
  }

 private:
  uint8_t _i2caddr{PCAL9535A_ADDRESS};
  WIRE& mWire;

  /**
   * Convert a given pin (0 - 15) to a port bit number (0 - 7)
   * \param pin Pin to convert to port bit number (0 - 15)
   * \return Port bit number (0 - 7)
   */
  uint8_t pinToBit(uint8_t pin) const
  {
    return pin % 8;
  }

  /**
   * Given a pin (0 - 15), select the register for the appropriate port
   * \param pin Pin to base decision on
   * \param port0 Register for Port0
   * \param port1 Register for Port1
   * \return Appropriate register (Port0 or Port1)
   */
  RegisterAddress pinToReg(uint8_t pin, RegisterAddress port0, RegisterAddress port1) const
  {
    return (pin < 8) ? port0 : port1;
  }

  /**
   * Read a register with a given address
   * \param register Address of register to read
   * \return Value of register
   */
  uint8_t readRegister(RegisterAddress reg)
  {
    // read the register
    mWire.beginTransmission(_i2caddr);
    mWire.write(static_cast<uint8_t>(reg));
    mWire.endTransmission();
    mWire.requestFrom(_i2caddr, uint8_t(1));
    return mWire.read();
  }

  /**
   * Write to a register with a given address
   * \param register Address of register to write
   * \param value Value to write to register
   */
  void writeRegister(RegisterAddress reg, uint8_t value)
  {
    // Write the register
    mWire.beginTransmission(_i2caddr);
    mWire.write(static_cast<uint8_t>(reg));
    mWire.write(value);
    mWire.endTransmission();
  }

  /**
   * Helper to update a single bit of an A/B register.
   * - Reads the current register value
   * - Writes the new register value
   * \param pin Pin for which the value shall be set
   * \param value Value to write (1 or 0)
   * \param port0 Register to write to if pin is in port0
   * \param port1 Register to write to if pin is in port1
   */
  void updateRegisterBit(uint8_t pin, uint8_t value, RegisterAddress port0, RegisterAddress port1)
  {
    const RegisterAddress regAddr = pinToReg(pin, port0, port1);
    uint8_t regValue = readRegister(regAddr);

    // set the value for the particular bit
    pca_bitWrite(regValue, pinToBit(pin), value);

    writeRegister(regAddr, regValue);
  }

};

} // namespace PCAL9535A

#undef pca_bitRead
#undef pca_bitSet
#undef pca_bitClear
#undef pca_bitWrite

#endif