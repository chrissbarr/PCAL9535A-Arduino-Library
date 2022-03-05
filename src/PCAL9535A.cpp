/*************************************************** 
 This is a library for the PCAL9535A I2C GPIO expander.

 Written by Chris Barr, 2019.
 ****************************************************/

#include "PCAL9535A.h"
#include <Arduino.h>
#include <Wire.h>

namespace PCAL9535A {

/**
 * Initializes the PCAL9535A given its HW address, see datasheet for address selection.
 */
void PCAL9535A::begin(uint8_t addr) {

	_i2caddr = constrain(addr, 0, 7);

	Wire.begin();

	// set all pins as inputs
	writeRegister(RegisterAddress::P0_CONFIG, 0xff);
	writeRegister(RegisterAddress::P1_CONFIG, 0xff);
}

/**
 * Initializes the PCAL9535A with 000 for the configurable part of the address
 */
void PCAL9535A::begin(void) {
	begin(0);
}

/**
 * Sets the pin mode to either INPUT or OUTPUT
 */
void PCAL9535A::pinMode(uint8_t pin, uint8_t mode) {
	updateRegisterBit(pin, (mode == INPUT ? 1 : 0), RegisterAddress::P0_CONFIG, RegisterAddress::P1_CONFIG);
}

/**
 * Read a single port, 0 or 1, and return its current 8 bit value.
 */
uint8_t PCAL9535A::readGPIO(uint8_t port) {
	Wire.beginTransmission(PCAL9535A_ADDRESS | _i2caddr);
	Wire.write(port == 0 ? RegisterAddress::P0_INPUT : RegisterAddress::P1_INPUT);
	Wire.endTransmission();
	Wire.requestFrom(PCAL9535A_ADDRESS | _i2caddr, 1);
	return Wire.read();
}

/**
 * Reads all 16 pins (port 0 and 1) into a single 16-bit variable.
 */
uint16_t PCAL9535A::readGPIO16() {
	uint16_t val = 0;
	uint8_t p0;

	// read the current GPIO inputs
	Wire.beginTransmission(PCAL9535A_ADDRESS | _i2caddr);
	Wire.write(RegisterAddress::P0_INPUT);
	Wire.endTransmission();

	Wire.requestFrom(PCAL9535A_ADDRESS | _i2caddr, 2);
	p0 = Wire.read();
	val = Wire.read();
	val <<= 8;
	val |= p0;

	return val;
}

/**
 * Write a single port.
 */
void PCAL9535A::writeGPIO(uint8_t port, uint8_t val) {
	Wire.beginTransmission(PCAL9535A_ADDRESS | _i2caddr);
	Wire.write(port == 0 ? RegisterAddress::P0_OUTPUT : RegisterAddress::P1_OUTPUT);
	Wire.write(val);
	Wire.endTransmission();
}

/**
 * Writes all the pins in one go. 
 */
void PCAL9535A::writeGPIO16(uint16_t val) {
	Wire.beginTransmission(PCAL9535A_ADDRESS | _i2caddr);
	Wire.write(RegisterAddress::P0_OUTPUT);
	Wire.write(val & 0xFF);
	Wire.write(val >> 8);
	Wire.endTransmission();
}

void PCAL9535A::digitalWrite(uint8_t pin, uint8_t val) {
	const RegisterAddress regAddr = pinToReg(pin, RegisterAddress::P0_OUTPUT, RegisterAddress::P1_OUTPUT);

	// read the current GPIO state, so we can modify only this one pin
	uint8_t gpio = readRegister(regAddr);

	// update the GPIO state with the pin we wish to set
	bitWrite(gpio, pinToBit(pin), val);

	// write the new GPIO state back out
	writeRegister(regAddr, gpio);
}

uint8_t PCAL9535A::digitalRead(uint8_t pin) {
	const RegisterAddress regAddr = pinToReg(pin, RegisterAddress::P0_INPUT, RegisterAddress::P1_INPUT);
	return (readRegister(regAddr) >> pinToBit(pin)) & 0x1;
}

/**
 * Set the pull-up/down resistor for a given pin
 */
void PCAL9535A::pinSetPull(uint8_t pin, PullSetting pull) {
	updateRegisterBit(pin, ((pull == PullSetting::NONE) ? RegisterValues_PULLENA::DISABLED : RegisterValues_PULLENA::ENABLED), RegisterAddress::P0_PULLENA, RegisterAddress::P1_PULLENA);
	updateRegisterBit(pin, ((pull == PullSetting::UP) ? RegisterValues_PULLSEL::PULLUP : RegisterValues_PULLSEL::PULLDOWN), RegisterAddress::P0_PULLSEL, RegisterAddress::P1_PULLSEL);	
}

/**
 * Sets the output pin drive strength
 */
void PCAL9535A::pinSetDriveStrength(uint8_t pin, DriveStrength strength) {
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
 	regValue |= (strength & 0x03) << ((pin % 4) * 2);

	writeRegister(regAddr, regValue);
}

/**
 * Sets the input pin polarity inversion
 */
void PCAL9535A::pinSetInputInversion(uint8_t pin, bool invert) {
	updateRegisterBit(pin, (invert ? 1 : 0), RegisterAddress::P0_POLINV, RegisterAddress::P1_POLINV);
}

/**
 * Sets the input pin polarity inversion
 */
void PCAL9535A::pinSetInputLatch(uint8_t pin, bool latch) {
	updateRegisterBit(pin, (latch ? 1 : 0), RegisterAddress::P0_ILATCH, RegisterAddress::P1_ILATCH);
}

/**
 * Enable / disable interrupts for a given pin.
 */
void PCAL9535A::pinSetInterruptEnabled(uint8_t pin, bool enabled) {
	updateRegisterBit(pin, (enabled ? 1 : 0), RegisterAddress::P0_INTMASK, RegisterAddress::P1_INTMASK);
}

uint8_t PCAL9535A::getLastInterruptPin() {
	uint8_t intf;

	intf = readRegister(RegisterAddress::P0_INTSTAT);
	for(int i = 0; i < 8; i++) {
		if (bitRead(intf, i)) {
			return i;
		}
	}

	intf = readRegister(RegisterAddress::P1_INTSTAT);
	for(int i = 0; i < 8; i++) {
		if (bitRead(intf, i)) {
			return i + 8;
		}
	}

	return PCAL9535A_INT_ERR;
}

uint8_t PCAL9535A::getInterruptPinValue() {
	uint8_t intPin = getLastInterruptPin();

	if(intPin != PCAL9535A_INT_ERR){
		return (digitalRead(intPin));
	}

	return PCAL9535A_INT_ERR;
}

void PCAL9535A::portSetOutputMode(uint8_t port, uint8_t mode) {
	uint8_t regValue = readRegister(RegisterAddress::OUTPUT_CONF);
	bitWrite(regValue, (port == 0 ? 0 : 1), (mode & 0x01));
	writeRegister(RegisterAddress::OUTPUT_CONF, regValue);
}

/**
 * Convert a given pin (0 - 15) to a port bit number (0 - 7)
 */
uint8_t PCAL9535A::pinToBit(uint8_t pin) const {
	return pin % 8;
}

/**
 * Select the register for port 0 or port 1 depending on the given pin
 */
RegisterAddress PCAL9535A::pinToReg(uint8_t pin, RegisterAddress port0, RegisterAddress port1) const {
	return (pin < 8) ? port0 : port1;
}

/**
 * Reads a given register
 */
uint8_t PCAL9535A::readRegister(RegisterAddress reg) {
	// read the current GPINTEN
	Wire.beginTransmission(PCAL9535A_ADDRESS | _i2caddr);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(PCAL9535A_ADDRESS | _i2caddr, 1);
	return Wire.read();
}

/**
 * Writes a given register
 */
void PCAL9535A::writeRegister(RegisterAddress reg, uint8_t regValue) {
	// Write the register
	Wire.beginTransmission(PCAL9535A_ADDRESS | _i2caddr);
	Wire.write(reg);
	Wire.write(regValue);
	Wire.endTransmission();
}

/**
 * Helper to update a single bit of an A/B register.
 * - Reads the current register value
 * - Writes the new register value
 */
void PCAL9535A::updateRegisterBit(uint8_t pin, uint8_t pValue, RegisterAddress port0, RegisterAddress port1) {
	const RegisterAddress regAddr = pinToReg(pin, port0, port1);
	uint8_t regValue = readRegister(regAddr);

	// set the value for the particular bit
	bitWrite(regValue, pinToBit(pin), pValue);
	writeRegister(regAddr, regValue);
}

} // namespace PCAL9535A