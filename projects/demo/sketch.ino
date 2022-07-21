// Headers
#include "sketch_only_statics.hpp"
#include "sketch.hpp"

// Communication
#include "powernet.h"
PowernetNamespace pntp;

// Services
void comms_thread_tty0(void * arg);
void comms_thread_eth0(void * arg);
void bms_thread(void * arg);

// Globals
bool application_is_running = true;

// Defines
#define PIN_LED_ACTIVITY PA9


// ----------------------------------------------------
//
static void catch_fault() {

	pinMode(PA8, OUTPUT);

    while(1) {
		serialEventUSB();

		digitalWrite(PA8, !digitalRead(PA8));
		delay(100);
    }
}

// ----------------------------------------------------
//
void comms_thread_eth0(void * arg) {

	pinMode(PIN_LED_ACTIVITY, OUTPUT);

	while(application_is_running) {

		// Listen for packets
		bool activity = pntp_listen_eth0();

		// Activity begin
		if(activity) {
			digitalWrite(PIN_LED_ACTIVITY, HIGH);
		}

		// Wait
		vTaskDelay(1);

        __asm__ volatile ("nop");

		// Activity cease
		digitalWrite(PIN_LED_ACTIVITY, LOW);
	}
}

// ----------------------------------------------------
//
void comms_thread_tty0(void * arg) {

	pinMode(PIN_LED_ACTIVITY, OUTPUT);

	while(application_is_running) {

		// Listen for packets
		bool activity = pntp_listen_tty0();

		// Activity begin
		if(activity) {
			digitalWrite(PIN_LED_ACTIVITY, HIGH);
		}

		// Wait
		vTaskDelay(1);

        __asm__ volatile ("nop");

		// Activity cease
		digitalWrite(PIN_LED_ACTIVITY, LOW);
	}
}


// ----------------------------------------------------
//
#define PIN_LED_0 A1

void bms_thread(void * arg) {

	// Init status LED
	pinMode(PIN_LED_0, OUTPUT);

	// Run the bms task forever
	while(application_is_running) {

		// Wait
		vTaskDelay(250);
	}
}


void setup() {

    // Keep alive
	pinMode(PA0, OUTPUT);
	pinMode(PA0, HIGH);

    // Launch program!
    pntp_begin("powernet demo");

	// Create worker task
    portBASE_TYPE bms_thread_pointer = xTaskCreate(
		bms_thread, 
		NULL, 
		3 * configMINIMAL_STACK_SIZE, 
		NULL, 
		1, 
		NULL
	);

	// Communication task 1
    portBASE_TYPE communication_thread_pointer_tty0 = xTaskCreate(
		comms_thread_tty0, 
		NULL, 
		5 * configMINIMAL_STACK_SIZE, 
		NULL, 
		1, 
		NULL
	);

	// Communication task 2
    portBASE_TYPE communication_thread_pointer_eth0 = xTaskCreate(
		comms_thread_eth0, 
		NULL, 
		5 * configMINIMAL_STACK_SIZE, 
		NULL, 
		1, 
		NULL
	);


    // Check for thread creation errors
    if (
		bms_thread_pointer != pdPASS
		|| communication_thread_pointer_tty0 != pdPASS
		|| communication_thread_pointer_eth0 != pdPASS
	) {
		catch_fault();
    }

    // Start scheduler
    vTaskStartScheduler();

    // We should not reach this point unless there was an error
    catch_fault();
};



void loop(){    

    // Never runs since we use threads
	//bool activity = pntp_listen();

};
