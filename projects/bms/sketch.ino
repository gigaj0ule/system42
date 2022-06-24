// Headers
#include "sketch.hpp"
#include "sketch_only_statics.hpp"
#include "Wire.h"

#include "protocol.hpp"

#if defined(USBD_USE_HID_COMPOSITE)
    #include "Keyboard.h"
    #include "Mouse.h"
#endif


TwoWire myWire (PB11, PB10);


#define PIN_POWER_ENABLE PA0
#define PIN_CELL_EXCITER PA10
#define PIN_CELL_V_ADC   PB1
#define PIN_DATA_BUS_0   PB12
#define PIN_DATA_BUS_1   PB13
#define PIN_DATA_BUS_2   PB14
#define PIN_DATA_BUS_3   PB15

void find_i2c_devices();


void data_bus_write (uint8_t address) {

	bool bit_0 = (address & 0b0001);
	bool bit_1 = (address & 0b0010);
	bool bit_2 = (address & 0b0100);
	bool bit_3 = (address & 0b1000);

	digitalWrite(PIN_DATA_BUS_0, bit_0);
	digitalWrite(PIN_DATA_BUS_1, bit_1);
	digitalWrite(PIN_DATA_BUS_2, bit_2);
	digitalWrite(PIN_DATA_BUS_3, bit_3);
}

int32_t get_cell_voltage(int cell_number) {
	data_bus_write(cell_number);
	os_delay(5);
	return analogRead(PIN_CELL_V_ADC);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setup() {

	// Power Enable
	pinMode(PIN_POWER_ENABLE, OUTPUT);
	digitalWrite(PIN_POWER_ENABLE, HIGH);

	// Data Bus Pins
	pinMode(PIN_DATA_BUS_0, OUTPUT_OPEN_DRAIN);
	pinMode(PIN_DATA_BUS_1, OUTPUT_OPEN_DRAIN);
	pinMode(PIN_DATA_BUS_2, OUTPUT_OPEN_DRAIN);
	pinMode(PIN_DATA_BUS_3, OUTPUT_OPEN_DRAIN);
	
	// ADC
	//pinMode(PIN_CELL_V_ADC, INPUT);

	// Exciter
	//pinMode(PIN_CELL_EXCITER, OUTPUT);
	//analogWriteFrequency(0.5 * 32768);
	//analogWrite(PIN_CELL_EXCITER, 128);

    // Init communication
	myWire.begin();

	pntp_begin("stm32 bonjour")
};


void loop(){    

	pntp_listen();

};


void find_i2c_devices()
{	
	/*
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
	send_event_to_host(pntp.scan_complete_event);
	*/
}
