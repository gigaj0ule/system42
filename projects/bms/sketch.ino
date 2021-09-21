// Headers
#include "sketch.hpp"
#include "sketch_only_statics.hpp"
#include "Wire.h"

#include "bq76952.h"

#include "protocol.hpp"

#if defined(USBD_USE_HID_COMPOSITE)
    #include "Keyboard.h"
    #include "Mouse.h"
#endif


extern TwoWire myWire;

//void find_i2c_devices();


static void worker_thread(void* arg) {
    while(true) {
	    // Do something every second
        os_delay(1000);
		//find_i2c_devices();
        //send_system_interrupt(SYSINT_USER_BUTTON_RISING);
		
		//BQ76952.setup();

		uint8_t pData[4] = {0};
		readRegister(LP_REGISTER_CTL_STATUS, pData, 2);
		
		//uint16_t ctl_status = 0;
		//BQ76952.getCtlStatus(&ctl_status);
		
		//uint8_t register_address = LP_REGISTER_CTL_STATUS;
		//i2c_tx(LP_I2C_ADDRESS, &register_address, 1);
    }
}

void setup() {

    #if defined(USBD_USE_HID_COMPOSITE)
        Mouse.begin();
        Keyboard.begin();
    #endif
        
    // Init communication
    early_setup();

	myWire.begin();

	BQ76952.begin(false);
    
    // Launch program!
    create_threads(worker_thread, 5, 3);
};


void loop(){    
    __asm__ volatile("nop");
    //os_delay(1);
    
    #if defined(USBD_USE_CDC)
    //SerialUSB.print("Hi");
    #endif
};


/*
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
*/