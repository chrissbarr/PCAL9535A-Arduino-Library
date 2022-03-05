/*************************************************** 
 This is a library for the PCAL9535A I2C GPIO expander.

 Written by Chris Barr, 2019.
 ****************************************************/

#ifndef _PCAL9535A_H_
#define _PCAL9535A_H_

#include <stdint.h>

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
  P25 = 0x00,
  P50 = 0x01,
  P75 = 0x10,
  P100 = 0x11
};

enum class RegisterValues_PULLENA : uint8_t  {
  DISABLED = 0x00,
  ENABLED = 0x01
};

enum class RegisterValues_PULLSEL : uint8_t  {
  PULLDOWN = 0x00,
  PULLUP = 0x01
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

class PCAL9535A {
public:
  /**
   * Initializes the PCAL9535A given its HW address, see datasheet for address selection.
   * /param addr Address of PCAL9535A (0 - 7)
   */
  void begin(HardwareAddress addr);

  /**
   * Initializes the PCAL9535A at the default address (0)
   */
  void begin();

  /**
   * Writes an 8-bit value to the entire port given.
   * \param port Port to write to (0 or 1)
   * \param value Value to write to port
   */
  void writeGPIO(Port port, uint8_t value);
  
  /**
   * Writes a 16-bit value to both ports.
   * \param value Value to write
   */
  void writeGPIO16(uint16_t value);

  /**
   * Reads all 16 pins (port 0 and 1) into a single 16-bit variable.
   * \return Variable containing all pin values as bits.
   */
  uint16_t readGPIO16();

  /**
   * Read a single port (0 or 1) and return its current 8 bit value.
   * \param port Port to read (0 or 1)
   * \return 8-bit value of port
   */
  uint8_t readGPIO(Port port);

  /**
   * Sets the mode of a given pin to either INPUT or OUTPUT
   * \param pin Pin to set (0 to 15)
   * \param mode Mode to set to (INPUT or OUTPUT)
   */
  void pinMode(uint8_t pin, uint8_t mode);

  /**
   * Sets the given pin (if configured as an output) to be either HIGH or LOW
   * \param pin Pin to set (0 to 15)
   * \param value Value to set to (LOW or HIGH)
   */
  void digitalWrite(uint8_t pin, uint8_t value);

  /**
   * Reads the given pin (if configured as an input) and returns the value
   * \param pin Pin to read (0 to 15)
   * \return Value of pin (LOW or HIGH)
   */
  uint8_t digitalRead(uint8_t pin);

  /**
   * Set pull-up/pull-down behaviour of a given pin
   * \param pin Pin to set behaviour for
   * \param pull Pullup/pulldown setting to apply
   */
  void pinSetPull(uint8_t pin, PullSetting pull);

  /**
   * Set drive strength behaviour of a given pin
   * \param pin Pin to set behaviour for
   * \param strength Drive strength setting to apply
   */
  void pinSetDriveStrength(uint8_t pin, DriveStrength strength);

  /**
   * Set input-inversion behaviour of a given pin
   * \param pin Pin to set behaviour for
   * \param invert Inversion setting to apply (pin input is inverted if true)
   */
  void pinSetInputInversion(uint8_t pin, bool invert);

  /**
   * Set input latching behaviour of a given pin
   * \param pin Pin to set behaviour for
   * \param latch Latch setting to apply (pin input is latching if true)
   */
  void pinSetInputLatch(uint8_t pin, bool latch);

  /**
   * Set interrupts enabled/disabled for a given pin
   * \param pin Pin to set behaviour for
   * \param enabled Enable / disable interrupts
   */
  void pinSetInterruptEnabled(uint8_t pin, bool enabled);

  /**
   * Get the last pin to trigger an interrupt
   * \return Pin which triggered interrupt
   */
  uint8_t getLastInterruptPin();

  /**
   * Get the value of the last pin to trigger an interrupt
   * \return Value of pin which triggered interrupt
   */
  uint8_t getInterruptPinValue();

  /**
   * Configure the port drive mode (either push-pull or open-drain)
   * \param port Port to apply the setting to
   * \param mode Mode to set the port to
   */
  void portSetOutputMode(Port port, DriveMode mode);

 private:
  uint8_t _i2caddr;

  /**
   * Convert a given pin (0 - 15) to a port bit number (0 - 7)
   * \param pin Pin to convert to port bit number (0 - 15)
   * \return Port bit number (0 - 7)
   */
  uint8_t pinToBit(uint8_t pin) const;

  /**
   * Given a pin (0 - 15), select the register for the appropriate port
   * \param pin Pin to base decision on
   * \param port0 Register for Port0
   * \param port1 Register for Port1
   * \return Appropriate register (Port0 or Port1)
   */
  RegisterAddress pinToReg(uint8_t pin, RegisterAddress port0, RegisterAddress port1) const;

  /**
   * Read a register with a given address
   * \param register Address of register to read
   * \return Value of register
   */
  uint8_t readRegister(RegisterAddress register);

  /**
   * Write to a register with a given address
   * \param register Address of register to write
   * \param value Value to write to register
   */
  void writeRegister(RegisterAddress register, uint8_t value);

  /**
   * Helper to update a single bit of an A/B register.
   * - Reads the current register value
   * - Writes the new register value
   * \param pin Pin for which the value shall be set
   * \param value Value to write (1 or 0)
   * \param port0 Register to write to if pin is in port0
   * \param port1 Register to write to if pin is in port1
   */
  void updateRegisterBit(uint8_t pin, uint8_t value, RegisterAddress port0, RegisterAddress port1);

};

} // namespace PCAL9535A

#endif