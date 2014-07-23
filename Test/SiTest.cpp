// Do not remove the include below
#include "SiTest.h"
#include "si4432.h"

#include "define.h"


Si4432 radio(7, 6);

//The setup function is called once at startup of the sketch
void setup() {

	Serial.begin(115200);
	delay(300);
	radio.init();
	radio.setBaudRate(35);
	radio.setFrequency(400);
	radio.readAll();

// Add your initialization code here
}

// The loop function is called in an endless loop
void loop() {
//Add your repeated code here

	byte dummy[15] = { 0x01, 0x3, 0x11, 0x13 };
#ifdef TX
	byte resLen = 0;
	byte answer[64] = {0};

	bool pkg = radio.sendPacket(6, dummy, 15, true, &resLen, answer);
#endif
#ifdef RX
	bool pkg = radio.waitForPacket(10);
#endif
	if (pkg) {
#ifdef TX
		Serial.print("PACKET CAME - ");
		Serial.println((int) resLen, DEC);

		for (byte i = 0; i < resLen; ++i) {
			Serial.print(answer[i], HEX);
			Serial.print(" ");
		}
		Serial.println();
#endif

#ifdef RX

		byte payLoad[64] = {0};
		byte len = 0;
		radio.getPacketReceived(&len, payLoad);
		Serial.print("PACKET CAME - ");
		Serial.println(len, DEC);

		for (byte i = 0; i < len; ++i) {
			Serial.print((int) payLoad[i], HEX);
			Serial.print(" ");
		}
		Serial.println(" ");

		Serial.print("Sending response- ");
		while (!radio.sendPacket(6, dummy, 15))
		;
		Serial.println(" SENT!");
#endif
	}

//delay(200);
}
