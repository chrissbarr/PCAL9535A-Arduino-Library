/*************************************************** 
 This is a library for the PCAL9535A I2C GPIO expander.

 Written by Chris Barr, 2019.
 ****************************************************/

#ifndef _PCAL9535A_H_
#define _PCAL9535A_H_

#include <Wire.h>

#define PCAL9535A_ADDRESS 0x20

// registers
#define PCAL9535A_P0_INPUT    0x00
#define PCAL9535A_P0_OUTPUT   0x02
#define PCAL9535A_P0_POLINV   0x04
#define PCAL9535A_P0_CONFIG   0x06
#define PCAL9535A_P0_DRVSTR1  0x40
#define PCAL9535A_P0_DRVSTR2  0x41
#define PCAL9535A_P0_ILATCH   0x44
#define PCAL9535A_P0_PULLENA  0x46
#define PCAL9535A_P0_PULLSEL  0x48
#define PCAL9535A_P0_INTMASK  0x4A
#define PCAL9535A_P0_INTSTAT  0x4C

#define PCAL9535A_P1_INPUT    0x01
#define PCAL9535A_P1_OUTPUT   0x03
#define PCAL9535A_P1_POLINV   0x05
#define PCAL9535A_P1_CONFIG   0x07
#define PCAL9535A_P1_DRVSTR1  0x42
#define PCAL9535A_P1_DRVSTR2  0x43
#define PCAL9535A_P1_ILATCH   0x45
#define PCAL9535A_P1_PULLENA  0x47
#define PCAL9535A_P1_PULLSEL  0x49
#define PCAL9535A_P1_INTMASK  0x4B
#define PCAL9535A_P1_INTSTAT  0x4D

#define PCAL9535A_OUTPUT_CONF 0x4F

#define PCAL9535A_DRVSTR_25   0x00
#define PCAL9535A_DRVSTR_50   0x01
#define PCAL9535A_DRVSTR_75   0x10
#define PCAL9535A_DRVSTR_100  0x11
#define PCAL9535A_PULLENA_DISABLED  0x00
#define PCAL9535A_PULLENA_ENABLED   0x01
#define PCAL9535A_PULLSEL_PULLDOWN  0x00
#define PCAL9535A_PULLSEL_PULLUP    0x01
#define PCAL9535A_OUTPUT_CONF_PP    0x00
#define PCAL9535A_OUTPUT_CONF_OD    0x01

#define PULL_NONE 0
#define PULL_UP   1
#define PULL_DOWN 2

#define PCAL9535A_INT_ERR 255

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
  void pinSetPull(uint8_t pin, uint8_t pull);
  void pinSetDriveStrength(uint8_t pin, uint8_t str);
  void pinSetInputInversion(uint8_t pin, bool invert);
  void pinSetInputLatch(uint8_t pin, bool latch);

  void pinSetInterruptEnabled(uint8_t pin, bool enabled);
  uint8_t getLastInterruptPin();
  uint8_t getInterruptPinValue();

  void portSetOutputMode(uint8_t port, uint8_t mode);

 private:
  uint8_t i2caddr;

  uint8_t pinToBit(uint8_t pin);
  uint8_t pinToReg(uint8_t pin, uint8_t port0addr, uint8_t port1addr);

  uint8_t readRegister(uint8_t addr);
  void writeRegister(uint8_t addr, uint8_t value);

  void updateRegisterBit(uint8_t p, uint8_t pValue, uint8_t port0addr, uint8_t port1addr);

};


#endif