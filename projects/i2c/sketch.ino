// Headers
#include "sketch_only_statics.hpp"

#include "sketch.hpp"

#include "powernet.h"
#include "Wire.h"

PowernetNamespace pntp;

TwoWire myWire (PB7, PB6);

void find_i2c_devices();

#define AFE_WAKE PB5
#define AFE_RESET PB4
#define AFE_ALERT PC15

void afe_wake(){
	pinMode(AFE_WAKE, OUTPUT);
	digitalWrite(AFE_WAKE, HIGH);
	delay(3);
	digitalWrite(AFE_WAKE, LOW);
}

void afe_reset() {
	pinMode(AFE_RESET, OUTPUT);
	digitalWrite(AFE_RESET, HIGH);
	delay(1);
	digitalWrite(AFE_RESET, LOW);
}

void setup() {

	myWire.begin();
	
    // Launch program!
    pntp_begin("i2c scanner");
};


void loop(){    

	if(pntp.afe_wake) {
		pntp.afe_wake = 0;
		afe_wake();
	}

	if(pntp.afe_reset) {
		pntp.afe_reset = 0;
		afe_reset();
	}

	pntp_listen();
	find_i2c_devices();
};


void find_i2c_devices()
{	
	volatile uint8_t error, address;

	pntp.i2c_device_count = 0;

	for(address = 1; address < 127; address++ ) {
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		while(myWire.available())
		{
			myWire.read();
		}
		
		myWire.beginTransmission(address);
		error = myWire.endTransmission();
		
		// Device address
		pntp.i2c_addresses[ (address * 6) + 0 ] = (address / 100) % 10 + 48;
		pntp.i2c_addresses[ (address * 6) + 1 ] = (address / 10 ) % 10 + 48;
		pntp.i2c_addresses[ (address * 6) + 2 ] = (address / 1  ) % 10 + 48;
		pntp.i2c_addresses[ (address * 6) + 3 ] = ':';

		// Newline
		pntp.i2c_addresses[ (address * 6) + 5 ] = ' ';

		switch(error) {

			// Success
			case 0:				
				pntp.i2c_addresses[ (address * 6) + 4 ] = 'Y';
				pntp.i2c_device_count++;
			break;

			// Buffer Overflow
			case 1:
				pntp.i2c_addresses[ (address * 6) + 4 ] = 'X';
			break;

			// NACK ADDR
			case 2:
				pntp.i2c_addresses[ (address * 6) + 4 ] = 'N';
			break;

			// NACK DATA
			case 3:
				pntp.i2c_addresses[ (address * 6) + 4 ] = 'D';
			break;

			// MISC
			case 4:
				pntp.i2c_addresses[ (address * 6) + 4 ] = '?';
			break;
		}
	}

	// Scan complete
	//send_event_to_host(pntp.scan_complete_event);
}
