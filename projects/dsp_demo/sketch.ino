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


// ---------------------------------------------------------------------------------
//

#include "dsp_functions.hpp"

// Filtering for DC_Cal(ibration)
//@todo make more easily configurable
#define calib_tau 0.2f  
static const float calib_filter_k = CURRENT_MEAS_PERIOD / calib_tau;

// These are the timings to be applied from the SVM calculator to the PWM timers
__attribute__((aligned(4))) uint32_t next_timings_stack_[2][6][3] = {0};

// Have the timings been used already?
bool next_timings_valid_[2] = {0};

// These are the timings currently active
__attribute__((aligned(4))) uint32_t active_timings_stack_[2][6][3] = {0};

// Global states used by the ADC callback
int pwm_adc_state_tracker_ = 0;

// These keep track of what SVM timings to apply to the PWM timers
uint8_t timer_one_SVM_timings_state_ = 0;
uint8_t timer_eight_SVM_timings_state_ = 0;
  
// Variables for holding ADC samples
float current_phB_[2] = 0.0f;
float current_phC_[2] = 0.0f;

float dc_offset_phB_[2] = {0.0f};
float dc_offset_phC_[2] = {0.0f};

// Sampled current measurements for the motor
float current_meas_[2][2][3] = {0.0f};
float buffered_current_meas_[2][2][3] = {0.0f};

// This is used to refrence the proper time sequence where measured current 
// values should be saved
uint8_t current_measurement_stack_index_ = 0;

// Setting this flag to true signals run_control_loop()
bool update_run_control_loop_ = false;

// Setting this flag to true does DC_Cal(ibration) for the active axis
bool do_dc_calibration_ = false;


// ---------------------------------------------------------------------------------
// @brief This function samples currents from ADC2 and ADC3
// @ingroup low_level_fast
static inline void sample_currents(bool timer_one) {
    
    // Storage variables for ADC values 
    uint32_t ADCValue_phB;
    uint32_t ADCValue_phC;

    // Check if this is an injected measurement and if so, get injected values from ADCs
    if (timer_one) {
        ADCValue_phB = ADC2->JDR1;
        ADCValue_phC = ADC3->JDR1;
    } 

    // Otherwise get regular values from ADCs
    else {
        ADCValue_phB = ADC2->DR;
        ADCValue_phC = ADC3->DR;
    }    

    // Convert motor currents to ADC values
    current_phB_[!timer_one] = ADCValue_phB; //axis_motor->phase_current_from_adcval(ADCValue_phB);
    current_phC_[!timer_one] = ADCValue_phC; //axis_motor->phase_current_from_adcval(ADCValue_phC);

    //axis_motor->current_meas_adc_samples_[current_measurement_stack_index_][0] = ADCValue_phB;
    //axis_motor->current_meas_adc_samples_[current_measurement_stack_index_][1] = ADCValue_phC;
}


// ---------------------------------------------------------------------------------
// @brief This is the callback from the ADC that we expect after the PWM has 
// triggered an ADC conversion. It is a low level state machine which keeps track of 
// all the most important step sequencing.
// @ingroup low_level_fast
// @param bool timer_one - Whether or not the callback is for an injected conversion 
//
//
// Most important state machine. Check it against this google sheet:
// https://docs.google.com/spreadsheets/d/1TYNPmy3y1iP_toVe8yVe_oJ2IY8obYzYNfZCcu-Vm9A/edit?usp=sharing
void dsp_state_machine(bool timer_one, int now) {

    // Sample number ++
    pntp.adc_sample_num ++;

    // Timer 1 triggers ADC2 on Injected, and ADC3 on Injected conversion
    // Timer 8 triggers ADC2 on Regular, and ADC3 on Regular conversion
    bool timer_eight = !timer_one;

    // 1 == downcounter
    volatile bool tim1_count_dir = (TIM1->CR1 & TIM_CR1_DIR);

    // Keep track of ADC state & reset when lost sync
    if(timer_one && (pwm_adc_state_tracker_ >= 12) && (tim1_count_dir == 0)) {

        pwm_adc_state_tracker_ = 0;
    }

    // Setting this flag to true takes a current measurement for the active axis
    bool sample_adc_ = false;

    if(timer_one) {
        // Timer One state machine
        switch(pwm_adc_state_tracker_) { 
            // Timer One Counting Up  
            case 0:
                // Update control loop for (axis0) on this state
                update_run_control_loop_ = true;        
                
                // Sample current on this state (axis0, newest [2])
                current_measurement_stack_index_ = 2;
                sample_adc_ = true;
                
                // Set the SVM timings state to (axis0, 1)
                timer_one_SVM_timings_state_ = 1;
                break;
            
            // Timer One Counting Down
            case 2:
                // Double-sample ADC on this state (axis0, oldest [0])
                current_measurement_stack_index_ = 0;           
                sample_adc_ = true;
                
                // Do DC bias measurement for (axis0)
                do_dc_calibration_ = true;

                // Set the SVM timings state to (axis0, 2)
                timer_one_SVM_timings_state_ = 2;
                break;
            
            // Timer One Counting Up
            case 4:
                // Sample current on this state (axis0, oldest [0])
                current_measurement_stack_index_ = 0;
                sample_adc_ = true;

                // Set the SVM timings state to (axis0, 3)
                timer_one_SVM_timings_state_ = 3;
                break;

            // Timer One Counting Down
            case 6:
                // Double-sample ADC on this state (axis0, middle [1])
                current_measurement_stack_index_ = 1;    
                sample_adc_ = true;

                // Set the SVM timings state to (axis0, 4)
                timer_one_SVM_timings_state_ = 4;
                break;

            // Timer One Counting Up
            case 8:
                // Sample current on this state (axis0, middle [1])        
                current_measurement_stack_index_ = 1;  
                sample_adc_ = true;
                
                // Set the SVM timings state to (axis0, 5)
                timer_one_SVM_timings_state_ = 5;
                break;

            // Timer One Counting Down
            case 10:
                // Double-sample ADC on this state (axis0, newest [2])
                current_measurement_stack_index_ = 2;  
                sample_adc_ = true;

                // Debugging
                // check_if_run_control_loop_failed(axis_motor);     

                // Buffer pwm timings 
                for(int i = 0; i < 6; i++) {
                    active_timings_stack_[0][i][0] = next_timings_stack_[0][i][0];
                    active_timings_stack_[0][i][1] = next_timings_stack_[0][i][1];
                    active_timings_stack_[0][i][2] = next_timings_stack_[0][i][2];
                }

                // Invalidate old timings
                next_timings_valid_[0] = false;
                
                // Set the SVM timings state to (axis0, 0)
                // right before the new current loop thread fires         
                timer_one_SVM_timings_state_ = 0;
                break;

            default:
                break;
        }
    }

    else {
        // Timer Eight state machine
        switch(pwm_adc_state_tracker_) {        

            // Timer Eight Counting Up
            case 5: 
                // Update control loop for (axis1) on this state
                update_run_control_loop_ = true;         
                
                // Sample current on this state (axis1, newest [2])        
                current_measurement_stack_index_ = 2;  
                sample_adc_ = true;            
                
                // Set the SVM timings state to (axis1, 1)
                timer_eight_SVM_timings_state_ = 1;
                break;

            // Timer Eight Counting Down
            case 7: 
                // Double-sample ADC on this state (axis1, oldest [0])
                current_measurement_stack_index_ = 0;     
                sample_adc_ = true;

                // Do DC bias measurement for (axis1)
                do_dc_calibration_ = true;
                
                // Set the SVM timings state to (axis1, 2)
                timer_eight_SVM_timings_state_ = 2; 
                break;

            // Timer Eight Counting Up
            case 9:
                // Sample current on this state (axis1, oldest [1])
                current_measurement_stack_index_ = 0; 
                sample_adc_ = true;
                
                // Set the SVM timings state to (axis1, 3)            
                timer_eight_SVM_timings_state_ = 3;

                // Update brake resistor
                //update_brake_current();
                break;

            // Timer Eight Counting Down
            case 11:
                // Double-sample ADC on this state (axis1, middle [1])
                sample_adc_ = true;

                // Set the SVM timings state to (axis1, 4)            
                timer_eight_SVM_timings_state_ = 4;   
                break;

            // Timer Eight Counting Up    
            case 1:
                // Sample current on this state (axis1, middle [0])
                current_measurement_stack_index_ = 1;
                sample_adc_ = true;

                // Set the SVM timings state to (axis1, 5)
                timer_eight_SVM_timings_state_ = 5;
                break;
            
            // Timer Eight Counting Down            
            case 3:
                // Double-sample ADC on this state (axis1, newest [2])
                current_measurement_stack_index_ = 2;       
                sample_adc_ = true;

                // Check whether current thread failed for (axis1)
                // check_if_run_control_loop_failed(axis_motor);
                // Double buffer the axis timings 
                for(int i = 0; i < 6; i++) {
                    active_timings_stack_[1][i][0] = next_timings_stack_[1][i][0];
                    active_timings_stack_[1][i][1] = next_timings_stack_[1][i][1];
                    active_timings_stack_[1][i][2] = next_timings_stack_[1][i][2];
                }

                // Invalidate old timings
                next_timings_valid_[1] = false;

                // Set the SVM timings state to (axis1, 0)
                // right before the new current loop thread fires         
                timer_eight_SVM_timings_state_ = 0;
                break;

            default:
                break;
        }
    }

    // Sample ADCs?
    if (sample_adc_) {

        // ADC2 and ADC3 record the phB and phC currents concurrently,
        // and their interrupts should arrive on the same clock cycle.
        // We dispatch the callbacks in order, so ADC2 will always be processed before ADC3.
        // Therefore we store the value from ADC2 and signal the thread that the
        // measurement is ready when we receive the ADC3 measurement
        
        // Get ADC samples
        sample_currents(timer_one);

        // Sample ADC for the active axis
        buffered_current_meas_[timer_eight][0][current_measurement_stack_index_] += current_phB_[timer_eight] - dc_offset_phB_[timer_eight];
        buffered_current_meas_[timer_eight][1][current_measurement_stack_index_] += current_phC_[timer_eight] - dc_offset_phC_[timer_eight];

        // Clear flag, nothing left to do!
        sample_adc_ = false;
    }
    
    // DC Bias Calculation
    if(do_dc_calibration_) {     
        
        // Take DC bias measurement, and apply low pass filter of about 200ms
        dc_offset_phB_[timer_eight] += (current_phB_[timer_eight] - dc_offset_phB_[timer_eight]) * calib_filter_k;
        dc_offset_phC_[timer_eight] += (current_phC_[timer_eight] - dc_offset_phC_[timer_eight]) * calib_filter_k;

        // Clear flag, nothing left to do!
        do_dc_calibration_ = false;
    }

    // Update PWM timing registers (we do this every state)
    if(timer_one) {
        TIM1->CCR1 = active_timings_stack_[0][timer_one_SVM_timings_state_][0];
        TIM1->CCR2 = active_timings_stack_[0][timer_one_SVM_timings_state_][1];
        TIM1->CCR3 = active_timings_stack_[0][timer_one_SVM_timings_state_][2];
    }
    else {
        TIM8->CCR1 = active_timings_stack_[1][timer_eight_SVM_timings_state_][0];
        TIM8->CCR2 = active_timings_stack_[1][timer_eight_SVM_timings_state_][1];
        TIM8->CCR3 = active_timings_stack_[1][timer_eight_SVM_timings_state_][2];
    }

    // Update the current control loop after all the new data has been had
    if(update_run_control_loop_) {
        
        // Get sensor samples as early as possible
        // Sample the hall effect sensor pins

        //for (int i = 0; i < num_GPIO; ++i) {
        //    GPIO_port_samples[timer_eight][i] = GPIOs_to_samp[i]->IDR;
        //}
       
        // Sample the encoder
        //axis.encoder_.sample_now();

        // Prepare hall readings
        // TODO move this to inside encoder update function
        //decode_hall_samples(axis.encoder_, GPIO_port_samples[timer_eight]);  

        // Cache current measurements so they don't get over-written
        for(int i = 0; i < 3; i++) {

            current_meas_[timer_eight][0][i] = buffered_current_meas_[timer_eight][0][i] / 2.0f;
            current_meas_[timer_eight][1][i] = buffered_current_meas_[timer_eight][1][i] / 2.0f;

            buffered_current_meas_[timer_eight][0][i] = 0.0f;
            buffered_current_meas_[timer_eight][1][i] = 0.0f;
        }

        // Trigger current control loop update
        //axis.signal_current_meas(); 
        
        // Clear flag, nothing left to do!
        update_run_control_loop_ = false;               
    }

    // Increment the state machine counter
    pwm_adc_state_tracker_ ++;
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
	DSP_set_adc_handler(&dsp_state_machine);
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
