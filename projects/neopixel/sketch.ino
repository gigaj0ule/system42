// Headers
#include "sketch.hpp"
#include "sketch_only_statics.hpp"

// Communication
#include "powernet.h"
PowernetNamespace pntp;

// Services
void comms_thread_tty0(const void * args);
void comms_thread_eth0(const void * args);
void program_thread(const void * args);

void catch_fault(void);

// Globals
bool application_is_running = true;

// Defines
#define PIN_LED_ACTIVITY PB6
#define PIN_LED_FAULT PB7

#include "Adafruit_NeoPixel.h"



//#############################################################################
// Definitions
//-----------------------------------------------------------------------------

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN      PA7 // On Trinket or Gemma, suggest changing this to 1

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMBER_OF_LEDS_TOTAL, PIN, NEO_RGB + NEO_KHZ800);


// ----------------------------------------------------
//
void setup() {

    // Keep alive
	pinMode(PA0, OUTPUT);
	pinMode(PA0, HIGH);

    // Power up LEDs
    pinMode(PA8, OUTPUT);
    digitalWrite(PA8, HIGH);

    // Init LEDs
    pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)

    // Launch program!
    pntp_begin("neopixel");

	// Create worker task
    osThreadDef(task_app, program_thread, osPriorityHigh, 0, 512 /* in 32-bit words */); 
    osThreadId thread_app = osThreadCreate(osThread(task_app), NULL);

	// Communication task 1
    osThreadDef(task_tty0, comms_thread_tty0, osPriorityAboveNormal, 0, 4000 /* in 32-bit words */); 
    osThreadId thread_tty0 = osThreadCreate(osThread(task_tty0), NULL);

	// Communication task 2
    osThreadDef(task_eth0, comms_thread_eth0, osPriorityAboveNormal, 0, 4000 /* in 32-bit words */); 
    osThreadId thread_eth0 = osThreadCreate(osThread(task_eth0), NULL);

    // Start scheduler
    osKernelStart();

    // We should not reach this point unless there was an error
    catch_fault();
};



void loop(){
    // Unused
};



// ----------------------------------------------------
//
void program_thread(void const * args) {

    pixels.clear(); // Set all pixel colors to 'off'

    while(true) {

        // For every bank...
        for(int j = 0; j < NUMBER_OF_LED_BANKS; j++) {

            uint16_t led_index = 0;

            // For every byte in a bank...
            for(int i = 0; i < NUMBER_OF_BYTES_PER_LED_BANK; i += 4) {

                // Get color from bytearray as uint32_t
                uint32_t this_color = (uint32_t) *((uint32_t *)(pntp.led_banks[j] + i));

                // Assign color to LED
                pixels.setPixelColor(led_index, this_color);

                // Index of LED array we're interested in
                led_index ++;
            }
        }

        // Depending on the type of LED chosen, switch what is used here
        //vTaskSuspendAll();
        pixels.show();   // Send the updated pixel colors to the hardware.
        //xTaskResumeAll();
        
        // Pass off this task
        osDelay(1);
    }
}



// ----------------------------------------------------
//
void comms_thread_eth0(void const * args) {

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
void comms_thread_tty0(void const * args) {

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

	pinMode(PIN_LED_FAULT, OUTPUT);

    while(1) {
		serialEventUSB();
		delay(100);

        digitalToggle(PIN_LED_FAULT);
    }
}
