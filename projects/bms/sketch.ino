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
#define PIN_DATA_BUS_0   PB4
#define PIN_DATA_BUS_1   PB5
#define PIN_DATA_BUS_2   PA15
#define PIN_DATA_BUS_3   PB3

void find_i2c_devices();


void data_bus_write (uint8_t address) {
	digitalWrite(PIN_DATA_BUS_0, (address & 0b0001) > 0);
	digitalWrite(PIN_DATA_BUS_1, (address & 0b0010) > 0);
	digitalWrite(PIN_DATA_BUS_2, (address & 0b0100) > 0);
	digitalWrite(PIN_DATA_BUS_3, (address & 0b1000) > 0);
}


float get_cell_voltage(int cell_number) {
	data_bus_write(cell_number);
}


static void worker_thread(void* arg) {
    while(true) {
	    // Do something every second
        os_delay(100);

		get_cell_voltage(0);
    }
}

void setup() {

    #if defined(USBD_USE_HID_COMPOSITE)
        Mouse.begin();
        Keyboard.begin();
    #endif

	// Power Enable
	pinMode(PIN_POWER_ENABLE, OUTPUT);
	digitalWrite(PIN_POWER_ENABLE, HIGH);

	// Data Bus Pins
	pinMode(PIN_DATA_BUS_0, OUTPUT);
	pinMode(PIN_DATA_BUS_1, OUTPUT);
	pinMode(PIN_DATA_BUS_2, OUTPUT);
	pinMode(PIN_DATA_BUS_3, OUTPUT);

	// Exciter
	pinMode(PIN_CELL_EXCITER, OUTPUT);
	analogWriteFrequency(32768);
	analogWrite(PIN_CELL_EXCITER, 128);

    // Init communication
    early_setup();

	myWire.begin();
    
    // Launch program!
    create_threads(worker_thread);
};


void loop(){    
    __asm__ volatile("nop");
};


void find_i2c_devices()
{	
	volatile uint8_t error, address;

	communicable.i2c_device_count = 0;

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
		communicable.i2c_addresses[ (address * 6) + 0 ] = (address / 100) % 10 + 48;
		communicable.i2c_addresses[ (address * 6) + 1 ] = (address / 10 ) % 10 + 48;
		communicable.i2c_addresses[ (address * 6) + 2 ] = (address / 1  ) % 10 + 48;
		communicable.i2c_addresses[ (address * 6) + 3 ] = ':';

		// Newline
		communicable.i2c_addresses[ (address * 6) + 5 ] = ' ';

		switch(error) {

			// Success
			case 0:				
				communicable.i2c_addresses[ (address * 6) + 4 ] = 'Y';
				communicable.i2c_device_count++;
			break;

			// Buffer Overflow
			case 1:
				communicable.i2c_addresses[ (address * 6) + 4 ] = 'X';
			break;

			// NACK ADDR
			case 2:
				communicable.i2c_addresses[ (address * 6) + 4 ] = 'N';
			break;

			// NACK DATA
			case 3:
				communicable.i2c_addresses[ (address * 6) + 4 ] = 'D';
			break;

			// MISC
			case 4:
				communicable.i2c_addresses[ (address * 6) + 4 ] = '?';
			break;
		}
	}

	// Scan complete
	send_event_to_host(communicable.scan_complete_event);
}
