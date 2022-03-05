/*************************************************** 
 This is a library for the PCAL9535A I2C GPIO expander.

 Written by Chris Barr, 2019.
 ****************************************************/

#include "PCAL9535A.h"
#include <Arduino.h>

/**
 * Initializes the PCAL9535A given its HW address, see datasheet for address selection.
 */
void PCAL9535A::begin(uint8_t addr) {

	_i2caddr = constrain(addr, 0, 7);

	Wire.begin();

	// set all pins as inputs
	writeRegister(PCAL9535A_P0_CONFIG, 0xff);
	writeRegister(PCAL9535A_P1_CONFIG, 0xff);
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
	updateRegisterBit(pin, (mode == INPUT ? 1 : 0), PCAL9535A_P0_CONFIG, PCAL9535A_P1_CONFIG);
}

/**
 * Read a single port, 0 or 1, and return its current 8 bit value.
 */
uint8_t PCAL9535A::readGPIO(uint8_t port) {
	Wire.beginTransmission(PCAL9535A_ADDRESS | _i2caddr);
	Wire.write(port == 0 ? PCAL9535A_P0_INPUT : PCAL9535A_P1_INPUT);
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
	Wire.write(PCAL9535A_P0_INPUT);
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
	Wire.write(port == 0 ? PCAL9535A_P0_OUTPUT : PCAL9535A_P1_OUTPUT);
	Wire.write(val);
	Wire.endTransmission();
}

/**
 * Writes all the pins in one go. 
 */
void PCAL9535A::writeGPIO16(uint16_t val) {
	Wire.beginTransmission(PCAL9535A_ADDRESS | _i2caddr);
	Wire.write(PCAL9535A_P0_OUTPUT);
	Wire.write(val & 0xFF);
	Wire.write(val >> 8);
	Wire.endTransmission();
}

void PCAL9535A::digitalWrite(uint8_t pin, uint8_t val) {
	uint8_t gpio;
	uint8_t bit = pinToBit(pin);
	uint8_t regAddr = pinToReg(pin, PCAL9535A_P0_OUTPUT, PCAL9535A_P1_OUTPUT);

	// read the current GPIO state, so we can modify only this one pin
	gpio = readRegister(regAddr);

	// update the GPIO state with the pin we wish to set
	bitWrite(gpio, bit, val);

	// write the new GPIO state back out
	writeRegister(regAddr, gpio);
}

uint8_t PCAL9535A::digitalRead(uint8_t pin) {
	uint8_t bit = pinToBit(pin);
	uint8_t regAddr = pinToReg(pin, PCAL9535A_P0_INPUT, PCAL9535A_P1_INPUT);
	return (readRegister(regAddr) >> bit) & 0x1;
}

/**
 * Set the pull-up/down resistor for a given pin
 */
void PCAL9535A::pinSetPull(uint8_t pin, uint8_t pull) {
	updateRegisterBit(pin, ((pull == PULL_NONE) ? PCAL9535A_PULLENA_DISABLED : PCAL9535A_PULLENA_ENABLED), PCAL9535A_P0_PULLENA, PCAL9535A_P1_PULLENA);
	updateRegisterBit(pin, ((pull == PULL_UP) ? PCAL9535A_PULLSEL_PULLUP : PCAL9535A_PULLSEL_PULLDOWN), PCAL9535A_P0_PULLSEL, PCAL9535A_P1_PULLSEL);	
}

/**
 * Sets the output pin drive strength
 */
void PCAL9535A::pinSetDriveStrength(uint8_t pin, uint8_t str) {
	uint8_t regAddr;
	uint8_t regValue;

	if (pinToBit(pin) < 4) {
		regAddr = pinToReg(pin, PCAL9535A_P0_DRVSTR1, PCAL9535A_P1_DRVSTR1);
	}
	else
	{
		regAddr = pinToReg(pin, PCAL9535A_P0_DRVSTR2, PCAL9535A_P1_DRVSTR2);
	}

 	regValue = readRegister(regAddr);
 	regValue |= (str & 0x03) << ((pin % 4) * 2);

	writeRegister(regAddr, regValue);
}

/**
 * Sets the input pin polarity inversion
 */
void PCAL9535A::pinSetInputInversion(uint8_t pin, bool invert) {
	updateRegisterBit(pin, (invert ? 1 : 0), PCAL9535A_P0_POLINV, PCAL9535A_P1_POLINV);
}

/**
 * Sets the input pin polarity inversion
 */
void PCAL9535A::pinSetInputLatch(uint8_t pin, bool latch) {
	updateRegisterBit(pin, (latch ? 1 : 0), PCAL9535A_P0_ILATCH, PCAL9535A_P1_ILATCH);
}

/**
 * Enable / disable interrupts for a given pin.
 */
void PCAL9535A::pinSetInterruptEnabled(uint8_t pin, bool enabled) {

	updateRegisterBit(pin, (enabled ? 1 : 0), PCAL9535A_P0_INTMASK, PCAL9535A_P1_INTMASK);
}

uint8_t PCAL9535A::getLastInterruptPin() {
	uint8_t intf;

	intf = readRegister(PCAL9535A_P0_INTSTAT);
	for(int i = 0; i < 8; i++) if (bitRead(intf, i)) return i;

	intf = readRegister(PCAL9535A_P1_INTSTAT);
	for(int i = 0; i < 8; i++) if (bitRead(intf, i)) return i + 8;

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
	uint8_t regAddr = PCAL9535A_OUTPUT_CONF;
	uint8_t regValue = readRegister(regAddr);
	bitWrite(regValue, (port == 0 ? 0 : 1), (mode & 0x01));
	writeRegister(regAddr, regValue);
}

/**
 * Convert a given pin (0 - 15) to a port bit number (0 - 7)
 */
uint8_t PCAL9535A::pinToBit(uint8_t pin) {
	return pin % 8;
}

/**
 * Select the register for port 0 or port 1 depending on the given pin
 */
uint8_t PCAL9535A::pinToReg(uint8_t pin, uint8_t port0addr, uint8_t port1addr) {
	return (pin < 8) ? port0addr : port1addr;
}

/**
 * Reads a given register
 */
uint8_t PCAL9535A::readRegister(uint8_t addr) {
	// read the current GPINTEN
	Wire.beginTransmission(PCAL9535A_ADDRESS | _i2caddr);
	Wire.write(addr);
	Wire.endTransmission();
	Wire.requestFrom(PCAL9535A_ADDRESS | _i2caddr, 1);
	return Wire.read();
}

/**
 * Writes a given register
 */
void PCAL9535A::writeRegister(uint8_t regAddr, uint8_t regValue) {
	// Write the register
	Wire.beginTransmission(PCAL9535A_ADDRESS | _i2caddr);
	Wire.write(regAddr);
	Wire.write(regValue);
	Wire.endTransmission();
}

/**
 * Helper to update a single bit of an A/B register.
 * - Reads the current register value
 * - Writes the new register value
 */
void PCAL9535A::updateRegisterBit(uint8_t pin, uint8_t pValue, uint8_t port0addr, uint8_t port1addr) {
	uint8_t regValue;
	uint8_t regAddr = pinToReg(pin, port0addr, port1addr);
	uint8_t bit = pinToBit(pin);
	regValue = readRegister(regAddr);

	// set the value for the particular bit
	bitWrite(regValue, bit, pValue);
	writeRegister(regAddr, regValue);
}