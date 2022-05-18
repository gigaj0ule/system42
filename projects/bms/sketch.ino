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

void find_i2c_devices();


static void worker_thread(void* arg) {
    while(true) {
	    // Do something every second
        os_delay(1000);
		find_i2c_devices();
        //send_system_interrupt(SYSINT_USER_BUTTON_RISING);
    }
}

void setup() {

    #if defined(USBD_USE_HID_COMPOSITE)
        Mouse.begin();
        Keyboard.begin();
    #endif

	pinMode(PC14, OUTPUT);
	digitalWrite(PC14, LOW);

	pinMode(PB12, OUTPUT);
	digitalWrite(PB12, HIGH);

	// Power Enable
	pinMode(PA0, OUTPUT);
	digitalWrite(PA0, HIGH);

    // Init communication
    early_setup();

	myWire.begin();
    
    // Launch program!
    create_threads(worker_thread);
};


void loop(){    
    __asm__ volatile("nop");
    //os_delay(1);
    
    #if defined(USBD_USE_CDC)
    //SerialUSB.print("Hi");
    #endif
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
