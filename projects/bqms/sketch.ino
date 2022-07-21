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
void comms_thread_tty0(void const * args);
void comms_thread_eth0(void const * args);
void bms_thread(const void * args);

void catch_fault(void);

bool application_is_running = true;




// ----------------------------------------------------
// Arduino.h run once
//
void setup() {

	pinMode(PA0, OUTPUT);
	pinMode(PA0, HIGH);

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
    osThreadDef(task_app, bms_thread, osPriorityHigh, 0, 512); 
    osThreadId thread_app = osThreadCreate(osThread(task_app), NULL);

	// Communication task 1
    osThreadDef(task_tty0, comms_thread_tty0, osPriorityAboveNormal, 0, 1024); 
    osThreadId thread_tty0 = osThreadCreate(osThread(task_tty0), NULL);

	// Communication task 2
    osThreadDef(task_eth0, comms_thread_eth0, osPriorityAboveNormal, 0, 1024); 
    osThreadId thread_eth0 = osThreadCreate(osThread(task_eth0), NULL);

    // Start scheduler
    osKernelStart();

    // We should not reach this point unless there was an error
    catch_fault();
}

// Arduino.h run Forever
//
void loop() {
	bool activity = pntp_listen();
}

// ----------------------------------------------------
// Observe Battery
//
#define PIN_LED_0 A1

void bms_thread(const void * arg) {

	// Init status LED
	pinMode(PIN_LED_0, OUTPUT);

	// Run the bms task forever
	while(application_is_running) {

		// Listen to BMS
		bq769x0_listen();

		// Scan I2C
		//find_i2c_devices();

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


// ----------------------------------------------------
//
#define PIN_LED_ACTIVITY PB6

static void comms_thread_eth0(void const * args) {

	pinMode(PIN_LED_ACTIVITY, OUTPUT);

	while(application_is_running) {

		// Listen for packets
		bool activity = pntp_listen_eth0();

		// Activity begin
		if(activity) {
			digitalWrite(PIN_LED_ACTIVITY, LOW);
		}

		// Wait
		osDelay(1);

		// Activity cease
		digitalWrite(PIN_LED_ACTIVITY, HIGH);
	}
}

// ----------------------------------------------------
//
static void comms_thread_tty0(void const * args) {

	pinMode(PIN_LED_ACTIVITY, OUTPUT);

	while(application_is_running) {

		// Listen for packets
		bool activity = pntp_listen_tty0();

		// Activity begin
		if(activity) {
			digitalWrite(PIN_LED_ACTIVITY, LOW);
		}

		// Wait
		osDelay(1);

		// Activity cease
		digitalWrite(PIN_LED_ACTIVITY, HIGH);
	}
}

// ----------------------------------------------------
//
void catch_fault() {

	pinMode(PA8, OUTPUT);

    while(1) {
		serialEventUSB();

		digitalWrite(PA8, !digitalRead(PA8));
		delay(100);
    }
}

// Scan the I2C Bus
// 
/*
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
*/