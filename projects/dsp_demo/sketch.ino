// Headers
#include "sketch.hpp"
#include "sketch_only_statics.hpp"

// Communication
#include "powernet.h"
PowernetNamespace pntp;

// DSP
#include "dsp_functions.hpp"

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

// ----------------------------------------------------
//
void dsp_adc_callback(bool timer_one) {
    
    if(timer_one) {
        pntp.adc_sample_number ++; 
    }
}

// ----------------------------------------------------
//
extern float buffered_current_meas_[2][2][6];

void dsp_timer_one_control_loop(void) {
    
    pntp.timer_one_loop_number ++; 

    for(int i = 0; i < 6; i++) {
        pntp.adc_samples[TIMER_ONE_ARRAY_INDEX][ADC2_ARRAY_INDEX][i] 
            = buffered_current_meas_[TIMER_ONE_ARRAY_INDEX][ADC2_ARRAY_INDEX][i];

        pntp.adc_samples[TIMER_ONE_ARRAY_INDEX][ADC3_ARRAY_INDEX][i] 
            = buffered_current_meas_[TIMER_ONE_ARRAY_INDEX][ADC3_ARRAY_INDEX][i];
    }
}

// ----------------------------------------------------
//
void dsp_timer_eight_control_loop(void) {
    // Pass
}


// ----------------------------------------------------
//
void setup() {

    // Keep alive
	pinMode(PA0, OUTPUT);
	pinMode(PA0, HIGH);

    // Power up Load Switch
    pinMode(PA8, OUTPUT);
    digitalWrite(PA8, HIGH);

	// Setup DSP
	DSP_set_adc_sample_complete_callback(&dsp_adc_callback);
	DSP_set_timer_one_control_loop_callback(&dsp_timer_one_control_loop);
	DSP_set_timer_eight_control_loop_callback(&dsp_timer_eight_control_loop);

    DSP_setup();

    // Launch program!
    pntp_begin("powernet_dsp");

	// Create worker task
    osThreadDef(task_app, program_thread, osPriorityHigh, 0, 512); 
    osThreadId thread_app = osThreadCreate(osThread(task_app), NULL);

	// Communication task 1
    osThreadDef(task_tty0, comms_thread_tty0, osPriorityAboveNormal, 0, 4000); 
    osThreadId thread_tty0 = osThreadCreate(osThread(task_tty0), NULL);

	// Communication task 2
    osThreadDef(task_eth0, comms_thread_eth0, osPriorityAboveNormal, 0, 4000); 
    osThreadId thread_eth0 = osThreadCreate(osThread(task_eth0), NULL);

    // Start scheduler
    osKernelStart();

    // We should not reach this point unless there was an error
    catch_fault();
};


void loop(){
    // Unused  
	//bool activity = pntp_listen_tty0();    
};


// ----------------------------------------------------
//
void program_thread(void const * args) {

    while(true) {

		// Do something here!
		__asm__ volatile ("nop");

        // Pass off this task
        osDelay(10);
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
		osDelay(5);

		// Activity cease
		digitalWrite(PIN_LED_ACTIVITY, HIGH);
	}
}

// ----------------------------------------------------
//
void catch_fault() {

	pinMode(PIN_LED_FAULT, OUTPUT);

    while(1) {
		delay(100);

        digitalToggle(PIN_LED_FAULT);
    }
}
