// Includes
#include "sketch_only_statics.hpp"
#include "sketch.hpp"

// Arduino Libraries
#include "Wire.h"

// FreeRTOS
#include "STM32FreeRTOSConfig.h"
#include "STM32FreeRTOS.h"

// IO Defines
#include "my_io.h"

// Communication
#include "powernet.h"
PowernetNamespace pntp;

// Include BMS Specific Functions
#include "SparkFun_bq769x0.h"
#include "bms_functions.h"

// Hardware
TwoWire myWire (PB7, PB6);

// Debug
void find_i2c_devices();

// Loop Counter
uint64_t loop_interval_timer; 

// Services
void comms_thread(void * arg);
void bms_thread(void * arg);
bool application_is_running = true;


static void catch_fault() {

	pinMode(PA8, OUTPUT);

    while(1) {
		digitalWrite(PA8, !digitalRead(PA8));
		delay(500);
    }
}


// ----------------------------------------------------
// Arduino.h run once
//
void setup() {

	// Initialize I2C Bus
	myWire.begin();
	
    // Launch program!
    pntp_begin("bq769x0 BMS");

	// Init BQ
	if(bq769x0_initBQ(myWire, PC15) == false) {
		// Serious failure, no BQ found
	}

	// This keeps track of the interval
	loop_interval_timer = millis();

	// Create worker task
    portBASE_TYPE bms_thread_pointer = xTaskCreate(
		bms_thread, 
		NULL, 
		5 * configMINIMAL_STACK_SIZE, 
		NULL, 
		1, 
		NULL
	);

	// Communication task
    portBASE_TYPE communication_thread_pointer = xTaskCreate(
		comms_thread, 
		NULL, 
		5 * configMINIMAL_STACK_SIZE, 
		NULL, 
		1, 
		NULL
	);

    // Check for thread creation errors
    if (
		bms_thread_pointer != pdPASS
		|| communication_thread_pointer != pdPASS
	) {
		catch_fault();
    }

    // Start scheduler
    vTaskStartScheduler();

    // We should not reach this point unless there was an error
    catch_fault();
}

// Arduino.h run Forever
//
void loop() {

}


// ----------------------------------------------------
// Listen for communication
//

#define PIN_LED_ACTIVITY PA9

void comms_thread(void * arg) {


	pinMode(PIN_LED_ACTIVITY, OUTPUT);


	while(application_is_running) {

		// Listen for packets
		bool activity = pntp_listen();

		// Activity begin
		if(activity) {
			digitalWrite(PIN_LED_ACTIVITY, HIGH);
		}

		// Wait
		vTaskDelay(1);

		// Activity cease
		digitalWrite(PIN_LED_ACTIVITY, LOW);
	}
}

// ----------------------------------------------------
// Observe Battery
//
#define PIN_LED_0 A1

void bms_thread(void * arg) {

	// Init status LED
	pinMode(PIN_LED_0, OUTPUT);

	// Run the bms task forever
	while(application_is_running) {

		// Listen to BMS
		bq769x0_listen();

		// Scan I2C
		find_i2c_devices();

		// Get cell voltages
		bq_readCellVoltages();

		// Get cell currents
		bq_readCellTemps();

		// Balance cells
		bq_balanceCells();
			
		// Get pack voltage
		pntp.b0_cell_voltage_total = bq769x0_readPackVoltage();

		// Debug
		pntp.loop_counter ++;

		// Blink LED
		digitalWrite(PIN_LED_0, HIGH);
		vTaskDelay(2);
		digitalWrite(PIN_LED_0, LOW);

		// Wait
		vTaskDelay(248);
	}
}



// Scan the I2C Bus
// 
void find_i2c_devices() {
	
	// SerialUSB.println("Looking for I2C Devices...");

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
				//SerialUSB.print("I2C Device Found on: ");
				//SerialUSB.println(address);
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
	// send_event_to_host(pntp.scan_complete_event);
}