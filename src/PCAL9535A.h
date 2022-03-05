/*************************************************** 
 This is a library for the PCAL9535A I2C GPIO expander.

 Written by Chris Barr, 2019.
 ****************************************************/

#ifndef _PCAL9535A_H_
#define _PCAL9535A_H_

#include <stdint.h>

namespace PCAL9535A {

constexpr int PCAL9535A_ADDRESS = 0x20;

// registers
enum RegisterAddress : uint8_t {
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

enum DriveStrength : uint8_t  {
  P25 = 0x00,
  P50 = 0x01,
  P75 = 0x10,
  P100 = 0x11
};

enum RegisterValues_PULLENA : uint8_t  {
  DISABLED = 0x00,
  ENABLED = 0x01
};

enum RegisterValues_PULLSEL : uint8_t  {
  PULLDOWN = 0x00,
  PULLUP = 0x01
};

enum DriveMode : uint8_t  {
  PUSHPULL = 0x00,
  OPENDRAIN = 0x01
};

enum class PullSetting : uint8_t  {
  NONE,
  UP,
  DOWN
};

constexpr int PCAL9535A_INT_ERR = 255;

class PCAL9535A {
public:
  void begin(uint8_t addr);
  void begin(void);
  void writeGPIO(uint8_t port, uint8_t);
  void writeGPIO16(uint16_t);
  uint16_t readGPIO16();
  uint8_t readGPIO(uint8_t port);
  void pinMode(uint8_t pin, uint8_t mode);
  void digitalWrite(uint8_t pin, uint8_t val);
  uint8_t digitalRead(uint8_t pin);
  void pinSetPull(uint8_t pin, PullSetting pull);
  void pinSetDriveStrength(uint8_t pin, DriveStrength strength);
  void pinSetInputInversion(uint8_t pin, bool invert);
  void pinSetInputLatch(uint8_t pin, bool latch);
  void pinSetInterruptEnabled(uint8_t pin, bool enabled);
  uint8_t getLastInterruptPin();
  uint8_t getInterruptPinValue();
  void portSetOutputMode(uint8_t port, DriveMode mode);
  
 private:
  uint8_t _i2caddr;

  uint8_t pinToBit(uint8_t pin) const;
  RegisterAddress pinToReg(uint8_t pin, RegisterAddress port0, RegisterAddress port1) const;
  uint8_t readRegister(RegisterAddress register);
  void writeRegister(RegisterAddress register, uint8_t value);
  void updateRegisterBit(uint8_t p, uint8_t pValue, RegisterAddress port0, RegisterAddress port1);

};

} // namespace PCAL9535A

#endif