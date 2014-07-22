// Do not remove the include below
#include "SiTest.h"
#include "si4432.h"

#define DEBUG

Si4432 radio(7, 6);

//The setup function is called once at startup of the sketch
void setup() {

	Serial.begin(115200);
	delay(100);
	radio.init();

	//radio.setFrequency(900000000);
	//radio.readAll();

// Add your initialization code here
}

// The loop function is called in an endless loop
void loop() {
//Add your repeated code here

	byte dummy[15] = { 0x5, 0x6 , 0x10, 0x88};

	byte resLen = 0;
	byte answer[10] = {0};

	bool pkg = radio.sendPacket(6, dummy, 15000, true, &resLen, answer);

	//bool pkg = radio.waitForPacket(5000);

	if(!pkg)
	{
		Serial.println("No answer!");
	} else
	{
		byte payLoad[64] = {0};
		byte len = 0;
		radio.getPacketReceived(&len, payLoad);
	}

	delay(200);
}
