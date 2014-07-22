#include "si4432.h"

#include <SPI.h>

#define DEBUG

Si4432::Si4432(uint8_t sdnPin, uint8_t InterruptPin) :
		_sdnPin(sdnPin), _intPin(InterruptPin), _freqCarrier(433000000), _freqChannel(0), _bps(100000), _packageSign(
				0xDEAD) { // default is 450 mhz

}

void Si4432::setFrequency(unsigned long baseFrequency) {

	_freqCarrier = baseFrequency;
	if ((baseFrequency < 240000000UL) || (baseFrequency > 930000000UL))
		return; // invalid frequency

	byte highBand = 0;
	if (baseFrequency >= 480000000) {
		highBand = 1;
	}

	double fPart = (baseFrequency / (10000000.0 * (highBand + 1))) - 24;

	uint8_t freqband = (uint8_t) fPart; // truncate the int

	uint16_t freqcarrier = (fPart - (double) freqband) * 64000;

	// sideband is always on (0x40) :
	byte vals[3] = { 0x40 | (highBand << 5) | (freqband & 0x3F), freqcarrier >> 8, freqcarrier & 0xFF };

	BurstWrite(REG_FREQBAND, vals, 3);

}

void Si4432::setCommsSignature(uint16_t signature) {
	_packageSign = signature;

	ChangeRegister(REG_TRANSMIT_HEADER1, _packageSign >> 8); // header (signature) byte 3 val
	ChangeRegister(REG_TRANSMIT_HEADER0, (_packageSign & 0xFF)); // header (signature) byte 2 val

	ChangeRegister(REG_CHECK_HEADER1, _packageSign >> 8); // header (signature) byte 3 val for receive checks
	ChangeRegister(REG_CHECK_HEADER0, (_packageSign & 0xFF)); // header (signature) byte 2 val for receive checks

#ifdef DEBUG
	Serial.println("Package signature is set!");
#endif
}

void Si4432::init() {

	if (_intPin != 0)
		pinMode(_intPin, INPUT_PULLUP);

	pinMode(_sdnPin, OUTPUT);
	digitalWrite(_sdnPin, HIGH); // keep reset pin high, so chip is turned off

	pinMode(SS, OUTPUT);
	digitalWrite(SS, HIGH); // set pin high, so chip would know we don't use it. - well, it's turned off anyway but...

	SPI.begin();
	SPI.setBitOrder(MSBFIRST);
	SPI.setClockDivider(SPI_CLOCK_DIV2); // 16/ 2 = 8 MHZ. Max. is 10 MHZ, so we're cool.
	SPI.setDataMode(SPI_MODE0);

#ifdef DEBUG
	Serial.println("SPI is initialized now.");
#endif

	hardReset();

}

void Si4432::boot() {

	ChangeRegister(REG_GPIO0_CONF, 0x0F); // tx/rx data clk pin
	ChangeRegister(REG_GPIO1_CONF, 0x00); // POR inverted pin
	ChangeRegister(REG_GPIO2_CONF, 0x1C); // clear channel pin

	ChangeRegister(REG_DATAACCESS_CONTROL, 0x8D); // enable rx packet handling, enable tx packet handling, enable CRC, use CRC-IBM
	ChangeRegister(REG_HEADER_CONTROL1, 0x0C); // no broadcast address control, enable check headers for bytes 0 & 1
	ChangeRegister(REG_HEADER_CONTROL2, 0x22);  // enable headers byte 3 & 2, no fixed package length, sync word 3 & 2
	ChangeRegister(REG_PREAMBLE_LENGTH, 0x08); // 8 * 4 bits = 32 bits (4 bytes) preamble length
	ChangeRegister(REG_PREAMBLE_DETECTION, 0x3A); // validate 7 * 4 bits of preamble  in a package
	ChangeRegister(REG_SYNC_WORD3, 0x2D); // sync byte 3 val
	ChangeRegister(REG_SYNC_WORD2, 0xD4); // sync byte 2 val

	ChangeRegister(REG_TX_POWER, 0x1F); // max power

	ChangeRegister(REG_MODULATION_MODE1, 0x0C);
	ChangeRegister(REG_MODULATION_MODE2, 0x63); // use FIFO Mode, GFSK

	ChangeRegister(REG_CHANNEL_STEPSIZE, 0x64); // each channel is of 1 Mhz interval

	setFrequency(_freqCarrier); // default freq
	setBaudRate(_bps); // default baud rate is 500kpbs
	setChannel(_freqChannel); // default channel is 0
	setCommsSignature(_packageSign); // default signature

}

bool Si4432::sendPacket(uint8_t length, const byte* data, uint64_t ackTimeout, bool waitResponse,
		uint8_t* responseLength, byte* responseBuffer) {

	ChangeRegister(REG_PKG_LEN, length);

	BurstWrite(REG_FIFO, data, length);

	ChangeRegister(REG_INT_ENABLE1, 0x04); // set interrupts on for package sent
	ChangeRegister(REG_INT_ENABLE2, 0x00); // set interrupts off for anything else
	//read interrupt registers to clean them
	ReadRegister(REG_INT_STATUS1);
	ReadRegister(REG_INT_STATUS2);

	switchMode(TXMode | Ready);

	uint64_t enterMillis = millis();

	while (millis() - enterMillis < ackTimeout) {
		if ((_intPin != 0) && (digitalRead(_intPin) != 0)) {
			continue;
		}

		byte intStatus = ReadRegister(REG_INT_STATUS1);
		ReadRegister(REG_INT_STATUS2);

		if (intStatus & 0x04) {
			switchMode(Ready);
#ifdef DEBUG
			Serial.print("Package sent! -- ");
			Serial.println(intStatus, HEX);
#endif
			// package sent. now, return true if not to wait ack, or wait ack (wait for packet only for 'remaining' amount of time)
			if (waitResponse) {
				if (waitForPacket(enterMillis + ackTimeout - millis())) {
					getPacketReceived(responseLength, responseBuffer);
					return true;
				} else
					return false;
			} else
				return true;
		}

	}

	//timeout occured.
#ifdef DEBUG
	Serial.println("Timeout in Transit -- ");
#endif
	switchMode(Ready);
	clearTxFIFO();

	return false;

}

bool Si4432::waitForPacket(uint64_t waitMs) {

	switchMode(RXMode | Ready);

	ChangeRegister(REG_INT_ENABLE1, 0x03); // set interrupts on for package received and CRC error
	ChangeRegister(REG_INT_ENABLE2, 0x00); // set other interrupts off
	//read interrupt registers to clean them
	ReadRegister(REG_INT_STATUS1);
	ReadRegister(REG_INT_STATUS2);

	uint64_t enterMillis = millis();

	while (millis() - enterMillis < waitMs) {
		if ((_intPin != 0) && (digitalRead(_intPin) != 0)) {
			continue;
		}
		// check for package received status at EzMAC status register
		byte intStat = ReadRegister(REG_INT_STATUS1);
		ReadRegister(REG_INT_STATUS2);

		if (intStat & 0x02) { //interrupt occured, check it && read the Interrupt Status1 register for 'valid packet'
			switchMode(Ready);
#ifdef DEBUG
			Serial.print("Packet detected -- ");
			Serial.println(intStat, HEX);
#endif
			return true;
		} else if (intStat & 0x01) { // packet crc error
			switchMode(Ready);
#ifdef DEBUG
			Serial.print("CRC Error in Packet detected!-- ");
			Serial.println(intStat, HEX);
#endif
			return false;
		}
	}
	//timeout occured.
#ifdef DEBUG
	Serial.println("Timeout in receive-- ");
#endif

	switchMode(Ready);
	clearRxFIFO();

	return false;
}

void Si4432::getPacketReceived(uint8_t* length, byte* readData) {

	*length = ReadRegister(REG_RECEIVED_LENGTH);

	BurstRead(REG_FIFO, readData, *length);

}

void Si4432::setChannel(byte channel) {

	ChangeRegister(REG_FREQCHANNEL, channel);

}

void Si4432::switchMode(byte mode) {
	ChangeRegister(REG_STATE, mode); // receive mode
	delay(5);
#ifdef DEBUG
	ReadRegister(REG_DEV_STATUS);
#endif
}

void Si4432::ChangeRegister(Registers reg, byte value) {
	BurstWrite(reg, &value, 1);
}

void Si4432::setBaudRate(uint64_t bps) {

	if ((bps > 256000) || (bps < 30000)) // less then 30kpbs is normalliy ok, but i must find the proper register to change first - TODO
		return;
		
	_bps = bps;
	uint16_t bpsRegVal = (bps / 1000000.0) * 65536;

	byte vals[] = { bpsRegVal >> 8, bpsRegVal & 0x0F };

	BurstWrite(REG_TX_DATARATE1, vals, 2);
}

byte Si4432::ReadRegister(Registers reg) {
	byte val = 0xFF;
	BurstRead(reg, &val, 1);
	return val;
}

void Si4432::BurstWrite(Registers startReg, const byte value[], uint8_t length) {

	byte regVal = (byte) startReg | 0x80; // set MSB

	digitalWrite(SS, LOW);
	SPI.transfer(regVal);

#ifdef DEBUG

	for (byte i = 0; i < length; ++i) {
		Serial.print("Writing: ");
		Serial.print((regVal != 0xFF ? (regVal + i) & 0x7F : 0x7F), HEX);
		Serial.print(" | ");
		Serial.println(value[i], HEX);
		SPI.transfer(value[i]);
	}
#endif

	digitalWrite(SS, HIGH);
}

void Si4432::BurstRead(Registers startReg, byte value[], uint8_t length) {

	byte regVal = (byte) startReg & 0x7F; // set MSB

	digitalWrite(SS, LOW);
	SPI.transfer(regVal);

#ifdef DEBUG

	for (byte i = 0; i < length; ++i) {
		value[i] = SPI.transfer(0xFF);
		Serial.print("Reading: ");
		Serial.print((regVal != 0xFF ? (regVal + i) & 0x7F : 0x7F), HEX);
		Serial.print(" | ");
		Serial.println(value[i], HEX);
	}
#endif

	digitalWrite(SS, HIGH);
}

void Si4432::readAll() {

	byte allValues[0x7F];

	BurstRead(REG_DEV_TYPE, allValues, 0x7F);

}

void Si4432::clearTxFIFO() {
	ChangeRegister(REG_OPERATION_CONTROL, 0x01);
	ChangeRegister(REG_OPERATION_CONTROL, 0x00);
}

void Si4432::clearRxFIFO() {
	ChangeRegister(REG_OPERATION_CONTROL, 0x02);
	ChangeRegister(REG_OPERATION_CONTROL, 0x00);
}

void Si4432::clearFIFO() {
	ChangeRegister(REG_OPERATION_CONTROL, 0x03);
	ChangeRegister(REG_OPERATION_CONTROL, 0x00);
}

void Si4432::softReset() {
	ChangeRegister(REG_STATE, 0x80);
	delay(10);
	boot();

}

void Si4432::hardReset() {

	digitalWrite(_sdnPin, HIGH); // turn off the chip now

	digitalWrite(_sdnPin, LOW); // turn off the chip now
	delay(20);

	boot();
}
