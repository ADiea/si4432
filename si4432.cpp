#include "si4432.h"

#include <SPI.h>

Si4432::Si4432(uint8_t sdnPin, uint8_t InterruptPin) :
		_highBand(0), _sdnPin(sdnPin), _intPin(InterruptPin), _freqband(0x55), _freqChannel(0), _freqcarrier(0), _packageSign(
				0xFFFF) { // default is 450 mhz

}

void Si4432::setFrequency(unsigned long baseFrequency) {

	if ((baseFrequency < 240000000UL) || (baseFrequency > 930000000UL))
		return; // invalid frequency

	_highBand = 0;
	if (baseFrequency >= 480000000) {
		_highBand = 1;
	}

	double fPart = (baseFrequency / (10000000.0 * (_highBand + 1))) - 24;

	_freqband = (uint8_t) fPart; // truncate the int

	_freqcarrier = (fPart - _freqband) * 64000;

	ChangeRegister(REG_FREQBAND, 0x40 | (_highBand << 6) | (_freqband & 0x3F)); // sideband is always on (0x40)
	ChangeRegister(REG_FREQCARRIER_H, _freqcarrier >> 8);
	ChangeRegister(REG_FREQCARRIER_L, _freqcarrier & 0x00FF);

}

void Si4432::setCommsSignature(uint16_t signature) {
	_packageSign = signature;
}

void Si4432::setup() {

	if (_intPin != 0)
		pinMode(_intPin, INPUT_PULLUP);

	pinMode(_sdnPin, OUTPUT);
	digitalWrite(_sdnPin, HIGH); // turn off the chip now

	pinMode(SS, OUTPUT);
	digitalWrite(SS, HIGH); // set pin high, so chip would know we don't use it. - well, it's turned off anyway but...

	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV2); // 16/ 2 = 8 MHZ. Max. is 10 MHZ, so we're cool.
	SPI.setDataMode(SPI_MODE0);

#if DEBUG
	Serial.print("SPI is initialized now.");
#endif

}

void Si4432::sendPacket(uint8_t length, const byte* data, uint64_t ackTimeout, bool waitForResponsePacket,
		uint8_t* responseLength, byte* responseBuffer) {
}

void Si4432::waitForPacket() {
}

bool Si4432::getPacketReceived(uint64_t waitMs, uint8_t* length, byte* readData) {
}

void Si4432::waitForInt(uint64_t timeout) {
}

void Si4432::setChannel(unsigned long channel) {

}

void Si4432::switchMode(AntennaMode mode) {
}

void Si4432::ChangeRegister(Registers reg, byte value) {

	byte regVal = (byte) reg | 0x80; // set MSB

	digitalWrite(SS, LOW);

	SPI.transfer(regVal);
	SPI.transfer(value);

	digitalWrite(SS, HIGH);
}

byte Si4432::ReadRegister(Registers reg) {
	byte regVal = (byte) reg & 0x7F; // clear MSB

	digitalWrite(SS, LOW);

	SPI.transfer(regVal);
	byte val = SPI.transfer(0xFF);

	digitalWrite(SS, HIGH);

	return val;
}
