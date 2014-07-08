/*
 * SI4432 library for Arduino - v0.1
 *
 * Please note that Library uses standart SS pin for NSEL pin on the chip. This is 53 for Mega, 10 for Uno.
 * NOTES:
 *
 * V0.1
 * * Library supports no `custom' changes and usages of GPIO pin. Modify/add/remove your changes if necessary
 * * Radio use variable packet field format with 4 byte address header, first data field as length. Change if necessary
 *
 * made by Ahmet (theGanymedes) Ipkin
 *
 * 2014
 */

#ifndef si4432_H_
#define si4432_H_
#include "Arduino.h"

class Si4432 {
public:

	Si4432(uint8_t sdnPin, uint8_t InterruptPin = 0); // when a InterruptPin is given, interrupts are checked with this pin - rather than SPI polling

	void setFrequency(unsigned long baseFrequency);
	void setChannel(unsigned long channel);
	inline void setCommsSignature(uint16_t signature); // used to 'sign' packets with a predetermined signature
	void setup(); // sets SPI and pins ready and boot the radio

	void sendPacket(uint8_t length, const byte* data, uint64_t ackTimeout, bool waitForResponsePacket = false,
			uint8_t *responseLength = 0, byte* responseBuffer = 0); // switches to Tx mode and sends the package, then optionally receives response package

	void waitForPacket(); // switch to Rx mode
	bool getPacketReceived(uint64_t waitMs, uint8_t* length, byte* readData); // wait for the 'valid' package to arrive and read from Rx FIFO - returns false if no package or erroneous package received

protected:
	enum AntennaMode {
		RXMode, TXMode, Idle, TuneMode
	};

	uint8_t _highBand;

	uint8_t _sdnPin, _intPin;
	uint8_t _freqband, _freqChannel;
	uint16_t _freqcarrier;
	uint16_t _packageSign;

	void waitForInt(uint64_t timeout);

	void switchMode(AntennaMode mode);

	enum Registers {
		REG_STATE = 0x07, REG_FREQBAND = 0x75, REG_FREQCARRIER_H = 0x76, REG_FREQCARRIER_L = 0x77,

	};

	void ChangeRegister(Registers reg, byte value);
	byte ReadRegister(Registers reg);
};

#endif /* si4432_H_ */
