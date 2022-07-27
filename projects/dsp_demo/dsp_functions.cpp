#include <Arduino.h>
#include <interrupt.h>

#include "dsp_functions.hpp"
#include "dsp_init.hpp"

#ifndef __STM32F4__
    #error "Only STM32F40x supported by dsp_functions.cpp"
#endif


// These are the timings to be applied from the SVM calculator to the PWM timers
__attribute__((aligned(4))) uint32_t next_timings_stack_[2][6][3] = {0};

// These are the timings currently active
__attribute__((aligned(4))) uint32_t active_timings_stack_[2][6][3] = {0};

// Have the timings been used already?
bool next_timings_valid_[2] = {0};

// Global states used by the ADC callback
int pwm_adc_state_tracker_ = 0;

// These keep track of what SVM timings to apply to the PWM timers
uint8_t timer_one_SVM_timings_state_ = 0;
uint8_t timer_eight_SVM_timings_state_ = 0;
  
// Variables for holding ADC samples
float adcval_ADC2_[2] = {0.0f};
float adcval_ADC3_[2] = {0.0f};

// Filtering for DC_Cal(ibration)
//@todo make more easily configurable
#define calib_tau 0.2f  
static const float calib_filter_k = CURRENT_MEAS_PERIOD / calib_tau;

// DC Offset
float dc_offset_ADC2_[2] = {0.0f};
float dc_offset_ADC3_[2] = {0.0f};

// Sampled current measurements for the motor
float current_meas_[2][2][3] = {0.0f};
float buffered_current_meas_[2][2][3] = {0.0f};

// This is used to refrence the proper time sequence where measured current 
// values should be saved
uint8_t adc_measurement_stack_index_ = 0;

// Setting this flag to true signals run_control_loop()
bool update_control_loop_ = false;

// Setting this flag to true does DC_Cal(ibration) for the active axis
bool do_dc_calibration_ = false;


// ---------------------------------------------------------------------------------
// @brief This function samples currents from ADC2 and ADC3
// @ingroup low_level_fast
static inline void sample_adcs(bool timer_one) {
    
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
    adcval_ADC2_[!timer_one] = ADCValue_phB; //axis_motor->phase_current_from_adcval(ADCValue_phB);
    adcval_ADC3_[!timer_one] = ADCValue_phC; //axis_motor->phase_current_from_adcval(ADCValue_phC);
}


// ---------------------------------------------------------------------------------
// @brief This is the callback from the ADC that we expect after the PWM has 
// triggered an ADC conversion. It is a low level state machine which keeps track of 
// all the most important step sequencing.
//
// @param bool timer_one - Whether or not the callback is for an injected conversion 
//
//
// Most important state machine. Check it against this google sheet:
// https://docs.google.com/spreadsheets/d/1TYNPmy3y1iP_toVe8yVe_oJ2IY8obYzYNfZCcu-Vm9A/edit?usp=sharing
//
void dsp_state_machine(bool timer_one) {

    // Sample number ++
    // pntp.adc_sample_num ++;

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
                // Update control loop for (timer_one) on this state
                update_control_loop_ = true;        
                
                // Sample ADC on this state (timer_one, newest [2])
                adc_measurement_stack_index_ = 2;
                sample_adc_ = true;
                
                // Set the SVM timings state to (timer_one, 1)
                timer_one_SVM_timings_state_ = 1;
                break;
            
            // Timer One Counting Down
            case 2:
                // Double-sample ADC on this state (timer_one, oldest [0])
                adc_measurement_stack_index_ = 0;           
                sample_adc_ = true;
                
                // Do DC bias measurement for (timer_one)
                do_dc_calibration_ = true;

                // Set the SVM timings state to (timer_one, 2)
                timer_one_SVM_timings_state_ = 2;
                break;
            
            // Timer One Counting Up
            case 4:
                // Sample ADC on this state (timer_one, oldest [0])
                adc_measurement_stack_index_ = 0;
                sample_adc_ = true;

                // Set the SVM timings state to (timer_one, 3)
                timer_one_SVM_timings_state_ = 3;
                break;

            // Timer One Counting Down
            case 6:
                // Double-sample ADC on this state (timer_one, middle [1])
                adc_measurement_stack_index_ = 1;    
                sample_adc_ = true;

                // Set the SVM timings state to (timer_one, 4)
                timer_one_SVM_timings_state_ = 4;
                break;

            // Timer One Counting Up
            case 8:
                // Sample ADC on this state (timer_one, middle [1])        
                adc_measurement_stack_index_ = 1;  
                sample_adc_ = true;
                
                // Set the SVM timings state to (timer_one, 5)
                timer_one_SVM_timings_state_ = 5;
                break;

            // Timer One Counting Down
            case 10:
                // Double-sample ADC on this state (timer_one, newest [2])
                adc_measurement_stack_index_ = 2;  
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
                
                // Set the SVM timings state to (timer_one, 0)
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
                // Update control loop for (timer_eight) on this state
                update_control_loop_ = true;         
                
                // Sample ADC on this state (timer_eight, newest [2])        
                adc_measurement_stack_index_ = 2;  
                sample_adc_ = true;            
                
                // Set the SVM timings state to (timer_eight, 1)
                timer_eight_SVM_timings_state_ = 1;
                break;

            // Timer Eight Counting Down
            case 7: 
                // Double-sample ADC on this state (timer_eight, oldest [0])
                adc_measurement_stack_index_ = 0;     
                sample_adc_ = true;

                // Do DC bias measurement for (timer_eight)
                do_dc_calibration_ = true;
                
                // Set the SVM timings state to (timer_eight, 2)
                timer_eight_SVM_timings_state_ = 2; 
                break;

            // Timer Eight Counting Up
            case 9:
                // Sample ADC on this state (timer_eight, oldest [1])
                adc_measurement_stack_index_ = 0; 
                sample_adc_ = true;
                
                // Set the SVM timings state to (timer_eight, 3)            
                timer_eight_SVM_timings_state_ = 3;

                // Update brake resistor
                //update_brake_current();
                break;

            // Timer Eight Counting Down
            case 11:
                // Double-sample ADC on this state (timer_eight, middle [1])
                sample_adc_ = true;

                // Set the SVM timings state to (timer_eight, 4)            
                timer_eight_SVM_timings_state_ = 4;   
                break;

            // Timer Eight Counting Up    
            case 1:
                // Sample ADC on this state (timer_eight, middle [0])
                adc_measurement_stack_index_ = 1;
                sample_adc_ = true;

                // Set the SVM timings state to (timer_eight, 5)
                timer_eight_SVM_timings_state_ = 5;
                break;
            
            // Timer Eight Counting Down            
            case 3:
                // Double-sample ADC on this state (timer_eight, newest [2])
                adc_measurement_stack_index_ = 2;       
                sample_adc_ = true;

                // Check whether current thread failed for (timer_eight)
                // check_if_run_control_loop_failed(axis_motor);
                // Double buffer the axis timings 
                for(int i = 0; i < 6; i++) {
                    active_timings_stack_[1][i][0] = next_timings_stack_[1][i][0];
                    active_timings_stack_[1][i][1] = next_timings_stack_[1][i][1];
                    active_timings_stack_[1][i][2] = next_timings_stack_[1][i][2];
                }

                // Invalidate old timings
                next_timings_valid_[1] = false;

                // Set the SVM timings state to (timer_eight, 0)
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
        sample_adcs(timer_one);

        // Sample ADC for the active axis
        buffered_current_meas_[timer_eight][0][adc_measurement_stack_index_] += adcval_ADC2_[timer_eight] - dc_offset_ADC2_[timer_eight];
        buffered_current_meas_[timer_eight][1][adc_measurement_stack_index_] += adcval_ADC3_[timer_eight] - dc_offset_ADC3_[timer_eight];

        // Clear flag, nothing left to do!
        sample_adc_ = false;
    }
    
    // DC Bias Calculation
    if(do_dc_calibration_) {     
        
        // Take DC bias measurement, and apply low pass filter of about 200ms
        dc_offset_ADC2_[timer_eight] += (adcval_ADC2_[timer_eight] - dc_offset_ADC2_[timer_eight]) * calib_filter_k;
        dc_offset_ADC3_[timer_eight] += (adcval_ADC3_[timer_eight] - dc_offset_ADC3_[timer_eight]) * calib_filter_k;

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
    if(update_control_loop_) {
        
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

            // Since we double-sample our ADC we should divide samples by 2.0f
            current_meas_[timer_eight][0][i] = buffered_current_meas_[timer_eight][0][i] / 2.0f;
            current_meas_[timer_eight][1][i] = buffered_current_meas_[timer_eight][1][i] / 2.0f;

            // And reset our measurements when we are done buffering them
            buffered_current_meas_[timer_eight][0][i] = 0.0f;
            buffered_current_meas_[timer_eight][1][i] = 0.0f;
        }

        // Trigger current control loop update
        //axis.signal_current_meas(); 
        
        // Clear flag, nothing left to do!
        update_control_loop_ = false;               
    }

    // Increment the state machine counter
    pwm_adc_state_tracker_ ++;
}


// ---------------------------------------------------------------------------
// Callback used for PWM State Machine
//
void (*adc_callback_)(bool);
//
//
void DSP_set_adc_handler(void (*adc_callback)(bool)) {
    adc_callback_ = *adc_callback;
}


// ----------------------------------------------------------------------------
// ADC_IRQHander
//
extern "C" {

    void ADC_IRQHandler(void) {

        // The HAL's ADC handling mechanism adds many clock cycles of overhead
        // So we bypass it and handle the logic ourselves.
        // ADC1: Injected channel?
        if(__FAST__HAL_ADC_GET_FLAG(ADC1, ADC_FLAG_JEOC)) {

            // Calculate vbus voltage
            // vbus_voltage_sense_calculation();

            // Clear interrupt flag & start next conversion
            __FAST__HAL_ADC_MODIFY_FLAG(ADC1, START_INJECTED_CONVERSION);
        }
        
        // ADC2: Injected channel?
        else if(__FAST__HAL_ADC_GET_FLAG(ADC2, ADC_FLAG_JEOC)) {

            // Advance DSP state machine
            dsp_state_machine(false);

            // Callback?
            if(adc_callback_){
                // Callback()
                adc_callback_(true);
            }

            // Clear interrupt flag & start next conversion
            __FAST__HAL_ADC_MODIFY_FLAG(ADC2, START_INJECTED_CONVERSION);        
        }

        // ADC2: Regular channel?
        else if(__FAST__HAL_ADC_GET_FLAG(ADC2, ADC_FLAG_EOC)) {

            // Advance DSP state machine
            dsp_state_machine(false);

            // Callback?
            if(adc_callback_){
                // Callback()
                adc_callback_(false);
            }

            // Clear interrupt flag & start next conversion
            __FAST__HAL_ADC_MODIFY_FLAG(ADC2, START_REGULAR_CONVERSION);        
        }
    }
}


// ---------------------------------------------------------------------------
//
//
void DSP_setup() {
    
    // M0
    DSP_TIM1_Init();

    // M0 Encoder
    DSP_TIM3_Init();

    #ifndef USE_SINGLE_AXIS
        // M1
        DSP_TIM8_Init();

        // M1 Encoder
        DSP_TIM4_Init();
    #endif 

    // OC
    // DSP_TIM2_Init();
    
    // Input Capture
    //DSP_TIM5_Init();

    // uLTick (already handled by arduino.h)
    // DSP_TIM13_Init();

    // ADC DMA
    DSP_DMA_Init();

    // ADC
    DSP_ADC1_Init();
    DSP_ADC2_Init();
    DSP_ADC3_Init();

    // Start
    DSP_PWM_INIT();
}