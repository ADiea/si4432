#include "si4432.h"

#include <SPI.h>

// #define DEBUG

//values here are kept in khz x 10 format (for not to deal with decimals) - look at AN440 page 26 for whole table
const uint16_t IFFilterTable[][2] = { { 322, 0x26 }, { 3355, 0x88 }, { 3618, 0x89 }, { 4202, 0x8A }, { 4684, 0x8B }, {
		5188, 0x8C }, { 5770, 0x8D }, { 6207, 0x8E } };

Si4432::Si4432(uint8_t sdnPin, uint8_t InterruptPin) :
		_sdnPin(sdnPin), _intPin(InterruptPin), _freqCarrier(433000000), _freqChannel(0), _kbps(100), _packageSign(
				0xDEAD) { // default is 450 mhz

}

void Si4432::setFrequency(unsigned long baseFrequencyMhz) {

	if ((baseFrequencyMhz < 240) || (baseFrequencyMhz > 930))
		return; // invalid frequency

	_freqCarrier = baseFrequencyMhz;
	byte highBand = 0;
	if (baseFrequencyMhz >= 480) {
		highBand = 1;
	}

	double fPart = (baseFrequencyMhz / (10 * (highBand + 1))) - 24;

	uint8_t freqband = (uint8_t) fPart; // truncate the int

	uint16_t freqcarrier = (fPart - freqband) * 64000;

	// sideband is always on (0x40) :
	byte vals[3] = { 0x40 | (highBand << 5) | (freqband & 0x3F), freqcarrier >> 8, freqcarrier & 0xFF };

	BurstWrite(REG_FREQBAND, vals, 3);

}

void Si4432::setCommsSignature(uint16_t signature) {
	_packageSign = signature;

	ChangeRegister(REG_TRANSMIT_HEADER3, _packageSign >> 8); // header (signature) byte 3 val
	ChangeRegister(REG_TRANSMIT_HEADER2, (_packageSign & 0xFF)); // header (signature) byte 2 val

	ChangeRegister(REG_CHECK_HEADER3, _packageSign >> 8); // header (signature) byte 3 val for receive checks
	ChangeRegister(REG_CHECK_HEADER2, (_packageSign & 0xFF)); // header (signature) byte 2 val for receive checks

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

	byte currentFix[] = { 0x80, 0x40, 0x7F };
	BurstWrite(REG_CHARGEPUMP_OVERRIDE, currentFix, 3); // refer to AN440 for reasons

	ChangeRegister(REG_GPIO0_CONF, 0x0F); // tx/rx data clk pin
	ChangeRegister(REG_GPIO1_CONF, 0x00); // POR inverted pin
	ChangeRegister(REG_GPIO2_CONF, 0x1C); // clear channel pin

	ChangeRegister(REG_AFC_TIMING_CONTROL, 0x02); // refer to AN440 for reasons
	ChangeRegister(REG_AFC_LIMITER, 0xFF); // write max value - excel file did that.
	ChangeRegister(REG_AGC_OVERRIDE, 0x60); // max gain control
	ChangeRegister(REG_AFC_LOOP_GEARSHIFT_OVERRIDE, 0x3C); // turn off AFC
	ChangeRegister(REG_DATAACCESS_CONTROL, 0xAD); // enable rx packet handling, enable tx packet handling, enable CRC, use CRC-IBM
	ChangeRegister(REG_HEADER_CONTROL1, 0x0C); // no broadcast address control, enable check headers for bytes 3 & 2
	ChangeRegister(REG_HEADER_CONTROL2, 0x22);  // enable headers byte 3 & 2, no fixed package length, sync word 3 & 2
	ChangeRegister(REG_PREAMBLE_LENGTH, 0x08); // 8 * 4 bits = 32 bits (4 bytes) preamble length
	ChangeRegister(REG_PREAMBLE_DETECTION, 0x3A); // validate 7 * 4 bits of preamble  in a package
	ChangeRegister(REG_SYNC_WORD3, 0x2D); // sync byte 3 val
	ChangeRegister(REG_SYNC_WORD2, 0xD4); // sync byte 2 val

	ChangeRegister(REG_TX_POWER, 0x1F); // max power

	ChangeRegister(REG_CHANNEL_STEPSIZE, 0x64); // each channel is of 1 Mhz interval

	setFrequency(_freqCarrier); // default freq
	setBaudRate(_kbps); // default baud rate is 100kpbs
	setChannel(_freqChannel); // default channel is 0
	setCommsSignature(_packageSign); // default signature

	switchMode(Ready | TuneMode);

}

bool Si4432::sendPacket(uint8_t length, const byte* data, uint64_t ackTimeout, bool waitResponse,
		uint8_t* responseLength, byte* responseBuffer) {

	clearTxFIFO();
	ChangeRegister(REG_PKG_LEN, length);

	BurstWrite(REG_FIFO, data, length);

	ChangeRegister(REG_INT_ENABLE1, 0x04); // set interrupts on for package sent
	ChangeRegister(REG_INT_ENABLE2, 0x00); // set interrupts off for anything else
	//read interrupt registers to clean them
	ReadRegister(REG_INT_STATUS1);
	ReadRegister(REG_INT_STATUS2);

	switchMode(TXMode | TuneMode | Ready);

	uint64_t enterMillis = millis();

	while (millis() - enterMillis < ackTimeout) {

		if ((_intPin != 0) && (digitalRead(_intPin) != 0)) {
			continue;
		}

		byte intStatus = ReadRegister(REG_INT_STATUS1);
		ReadRegister(REG_INT_STATUS2);

		if (intStatus & 0x04) {
			delay(2);
			switchMode(TuneMode | Ready);
#ifdef DEBUG
			Serial.print("Package sent! -- ");
			Serial.println(intStatus, HEX);
#endif
			// package sent. now, return true if not to wait ack, or wait ack (wait for packet only for 'remaining' amount of time)
			if (waitResponse) {
				if (intStatus & 0x02) { // if there is a packet received
					getPacketReceived(responseLength, responseBuffer);
					softReset();
				} else if (waitForPacket(ackTimeout)) {
					getPacketReceived(responseLength, responseBuffer);
					softReset();
					return true;
				} else {
					return false;
				}
			} else {
				return true;
			}
		}
	}

	//timeout occured.
//#ifdef DEBUG
	Serial.println("Timeout in Transit -- ");
//#endif
	switchMode(TuneMode | Ready);

	if (ReadRegister(REG_DEV_STATUS) & 0x80) {
		clearFIFO();
	}

	return false;

}

bool Si4432::waitForPacket(uint64_t waitMs) {

	clearRxFIFO();

	ChangeRegister(REG_INT_ENABLE1, 0x03); // set interrupts on for package received and CRC error
	ChangeRegister(REG_INT_ENABLE2, 0xc0); // set other interrupts off
	//read interrupt registers to clean them
	ReadRegister(REG_INT_STATUS1);
	ReadRegister(REG_INT_STATUS2);

	switchMode(TuneMode | RXMode | Ready);
	//Serial.print("RECV MODE: ");
	//Serial.println(ReadRegister(REG_STATE), HEX);

	uint64_t enterMillis = millis();
	while (millis() - enterMillis < waitMs) {

		if ((_intPin != 0) && (digitalRead(_intPin) != 0)) {
			continue;
		}
		// check for package received status interrupt register
		byte intStat = ReadRegister(REG_INT_STATUS1);
		byte intStat2 = ReadRegister(REG_INT_STATUS2);

		if (intStat2 & 0x40) { //interrupt occured, check it && read the Interrupt Status1 register for 'preamble '

#ifdef DEBUG
			Serial.print("HEY!! HEY!! Valid Preamble detected -- ");
			Serial.println(intStat2, HEX);
#endif
		}

		if (intStat2 & 0x80) { //interrupt occured, check it && read the Interrupt Status1 register for 'preamble '

#ifdef DEBUG
			Serial.print("HEY!! HEY!! SYNC WORD detected -- ");
			Serial.println(intStat2, HEX);
#endif
		}

		if (intStat & 0x02) { //interrupt occured, check it && read the Interrupt Status1 register for 'valid packet'

//#ifdef DEBUG
			Serial.print("Packet detected -- ");
			Serial.println(intStat, HEX);
//#endif
			return true;
		} else if (intStat & 0x01) { // packet crc error

//#ifdef DEBUG
			Serial.print("CRC Error in Packet detected!-- ");
			Serial.println(intStat, HEX);
//#endif
			clearRxFIFO();
			return false;
		}
	}
	//timeout occured.

	Serial.println("Timeout in receive-- ");


	switchMode(TuneMode | Ready);
	//clearRxFIFO();

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
	//delay(20);
#ifdef DEBUG
	byte val = ReadRegister(REG_DEV_STATUS);
	if (val == 0 || val == 0xFF) {
		Serial.print(val, HEX);
		Serial.println(" -- WHAT THE HELL!!");
	}
#endif
}

void Si4432::ChangeRegister(Registers reg, byte value) {
	BurstWrite(reg, &value, 1);
}

void Si4432::setBaudRate(uint16_t kbps) {

	// chip normally supports very low bps values, but they are cumbersome to implement - so I just didn't implement lower bps values
	if ((kbps > 256) || (kbps < 1))
		return;
	_kbps = kbps;

	byte freqDev = kbps <= 10 ? 15 : 150;		// 15khz / 150 khz
	byte modulationValue = _kbps < 30 ? 0x4c : 0x0c;		// use FIFO Mode, GFSK, low baud mode on / off

	byte modulationVals[] = { modulationValue, 0x23, round((freqDev * 1000.0) / 625.0) }; // msb of the kpbs to 3rd bit of register
	BurstWrite(REG_MODULATION_MODE1, modulationVals, 3);

	// set data rate
	uint16_t bpsRegVal = round((kbps * (kbps < 30 ? 2097152 : 65536.0)) / 1000.0);
	byte datarateVals[] = { bpsRegVal >> 8, bpsRegVal & 0xFF };

	BurstWrite(REG_TX_DATARATE1, datarateVals, 2);

	//now set the timings
	uint16_t minBandwidth = (2 * (uint32_t) freqDev) + kbps;
#ifdef DEBUG
	Serial.print("min Bandwidth value: ");
	Serial.println(minBandwidth, HEX);
#endif
	byte IFValue = 0xff;
	//since the table is ordered (from low to high), just find the 'minimum bandwith which is greater than required'
	for (byte i = 0; i < 8; ++i) {
		if (IFFilterTable[i][0] >= (minBandwidth * 10)) {
			IFValue = IFFilterTable[i][1];
			break;
		}
	}
#ifdef DEBUG
	Serial.print("Selected IF value: ");
	Serial.println(IFValue, HEX);
#endif

	ChangeRegister(REG_IF_FILTER_BW, IFValue);

	byte dwn3_bypass = (IFValue & 0x80) ? 1 : 0; // if msb is set
	byte ndec_exp = (IFValue >> 4) & 0x07; // only 3 bits

	uint16_t rxOversampling = round((500.0 * (1 + 2 * dwn3_bypass)) / ((pow(2, ndec_exp - 3)) * (double ) kbps));

	uint32_t ncOffset = ceil(((double) kbps * (pow(2, ndec_exp + 20))) / (500.0 * (1 + 2 * dwn3_bypass)));

	uint16_t crGain = 2 + ((65535 * (int64_t) kbps) / ((int64_t) rxOversampling * freqDev));
	byte crMultiplier = 0x00;
	if (crGain > 0x7FF) {
		crGain = 0x7FF;
	}
#ifdef DEBUG
	Serial.print("dwn3_bypass value: ");
	Serial.println(dwn3_bypass, HEX);
	Serial.print("ndec_exp value: ");
	Serial.println(ndec_exp, HEX);
	Serial.print("rxOversampling value: ");
	Serial.println(rxOversampling, HEX);
	Serial.print("ncOffset value: ");
	Serial.println(ncOffset, HEX);
	Serial.print("crGain value: ");
	Serial.println(crGain, HEX);
	Serial.print("crMultiplier value: ");
	Serial.println(crMultiplier, HEX);

#endif

	byte timingVals[] = { rxOversampling & 0x00FF, ((rxOversampling & 0x0700) >> 3) | ((ncOffset >> 16) & 0x0F),
			(ncOffset >> 8) & 0xFF, ncOffset & 0xFF, ((crGain & 0x0700) >> 8) | crMultiplier, crGain & 0xFF };

	BurstWrite(REG_CLOCK_RECOVERY_OVERSAMPLING, timingVals, 6);

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

	for (byte i = 0; i < length; ++i) {
#ifdef DEBUG
		Serial.print("Writing: ");
		Serial.print((regVal != 0xFF ? (regVal + i) & 0x7F : 0x7F), HEX);
		Serial.print(" | ");
		Serial.println(value[i], HEX);
#endif
		SPI.transfer(value[i]);

	}

	digitalWrite(SS, HIGH);
}

void Si4432::BurstRead(Registers startReg, byte value[], uint8_t length) {

	byte regVal = (byte) startReg & 0x7F; // set MSB

	digitalWrite(SS, LOW);
	SPI.transfer(regVal);

	for (byte i = 0; i < length; ++i) {
		value[i] = SPI.transfer(0xFF);

#ifdef DEBUG
		Serial.print("Reading: ");
		Serial.print((regVal != 0x7F ? (regVal + i) & 0x7F : 0x7F), HEX);
		Serial.print(" | ");
		Serial.println(value[i], HEX);
#endif
	}

	digitalWrite(SS, HIGH);
}

void Si4432::readAll() {

	byte allValues[0x7F];

	BurstRead(REG_DEV_TYPE, allValues, 0x7F);

	for (byte i = 0; i < 0x7f; ++i) {
		Serial.print("REG(");
		Serial.print((int) REG_DEV_TYPE + i, HEX);
		Serial.print(") : ");
		Serial.println((int) allValues[i], HEX);
	}

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
	delay(20);
	boot();

}

void Si4432::hardReset() {

	digitalWrite(_sdnPin, HIGH); // turn off the chip now

	digitalWrite(_sdnPin, LOW); // turn off the chip now
	delay(50);

	boot();
}
