#pragma GCC optimize ("O3")

/* Includes ------------------------------------------------------------------*/

// Because of broken cmsis_os.h, we need to include arm_math first,
// otherwise chip specific defines are ommited
#include <stm32f405xx.h>
#include <stm32f4xx_hal.h>  // Sets up the correct chip specifc defines required by arm_math
#include <stm32f4xx_hal_tim.h>  // Sets up the correct chip specifc defines required by arm_math

#define ARM_MATH_CM4
#include <arm_math.h>

#include <cmsis_os.h>
#include <math.h>

#include <adc.h>
#include <gpio.h>
#include <main.h>
#include <tim.h>
#include <utils.h>

#include "odrive_main.h"


// These are slightly faster versions of the HAL functions which expect a 
// static argument
#define __FAST__HAL_ADC_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->SR) & (__FLAG__)) == (__FLAG__))
#define __FAST__HAL_ADC_CLEAR_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->SR) = ~(__FLAG__))
#define __FAST__HAL_ADC_MODIFY_FLAG(__HANDLE__, __FLAG__) (((__HANDLE__)->SR) = (__FLAG__))

#define __FAST__HAL_TIM_CLEAR_IT(__HANDLE__, __INTERRUPT__)  ((__HANDLE__)->SR = ~(__INTERRUPT__))
#define __FAST__HAL_TIM_GET_FLAG(__HANDLE__, __FLAG__)       (((__HANDLE__)->SR &(__FLAG__)) == (__FLAG__))

// Static arguments for the fast HAL functions
static constexpr const uint32_t START_INJECTED_CONVERSION = ~(ADC_FLAG_JSTRT | ADC_FLAG_JEOC);
static constexpr const uint32_t START_REGULAR_CONVERSION = ~(ADC_FLAG_STRT | ADC_FLAG_EOC);

// Global states used by the ADC callback
int pwm_adc_state_tracker_ = 0;

// These keep track of what SVM timings to apply to the PWM timers
uint8_t axis0_SVM_timings_state_ = 0;
uint8_t axis1_SVM_timings_state_ = 0;
  
// Variables for holding calculated current samples
float current_phB_ = 0.0f;
float current_phC_ = 0.0f;

// This is used to refrence the proper time sequence where measured current 
// values should be saved
uint8_t current_measurement_stack_index_ = 0;

// Setting this flag to true signals run_control_loop()
bool update_run_control_loop_ = false;

// Setting this flag to true does DC_Cal(ibration) for the active axis
bool do_dc_calibration_ = false;

// Filtering for DC_Cal(ibration)
//@todo make more easily configurable
#define calib_tau 0.2f  
static const float calib_filter_k = current_meas_period / calib_tau;

// Two motors, sampling port A,B,C (coherent with current meas timing)
static const GPIO_TypeDef* GPIOs_to_samp[] = { GPIOA, GPIOB, GPIOC };
static const int num_GPIO = sizeof(GPIOs_to_samp) / sizeof(GPIOs_to_samp[0]); 

// Samples of the GPIO port (for hall sensors, encoder)
static uint16_t GPIO_port_samples [2][num_GPIO];

// Constants used for calculation
//static constexpr const int adc_full_scale = ADC_FULL_SCALE;
//static constexpr const int adc_half_scale = ADC_HALF_SCALE;
//static constexpr const float adc_ref_voltage = ADC_REF_VOLTAGE;
//static constexpr const float adc_volts_per_count = ADC_VOLTS_PER_COUNT;
static constexpr const float adc_vbus_voltage_scale = ADC_VBUS_VOLTAGE_SCALE;
static constexpr const uint16_t clocks_per_cnt = (uint16_t)((float)TIM_1_8_CLOCK_HZ / (float)TIM_APB1_CLOCK_HZ);

// This value is updated by the DC-bus reading ADC.
// Arbitrary non-zero inital value to avoid division by zero if ADC reading is late
float vbus_voltage = 12.0f;

// @brief This function decodes hall effect samples.
// @ingroup low_level_fast
static void decode_hall_samples(Encoder& enc, uint16_t GPIO_samples[num_GPIO]) {

    GPIO_TypeDef* hall_ports[] = {
        enc.hw_config_.hallC_port,
        enc.hw_config_.hallB_port,
        enc.hw_config_.hallA_port,
    };

    uint16_t hall_pins[] = {
        enc.hw_config_.hallC_pin,
        enc.hw_config_.hallB_pin,
        enc.hw_config_.hallA_pin,
    };

    uint8_t hall_state = 0x0;
    for (int i = 0; i < 3; ++i) {
        int port_idx = 0;
        for (;;) {
            auto port = GPIOs_to_samp[port_idx];
            if (port == hall_ports[i])
                break;
            ++port_idx;
        }

        hall_state <<= 1;
        hall_state |= (GPIO_samples[port_idx] & hall_pins[i]) ? 1 : 0;
    }

    enc.hall_state_ = hall_state;
}

// @brief Halts interrupts for critical task completion
// @note These functions enable and disable interrupts (just like their cousins in low_level.h) 
// but must be a static part of this file to be truly inlined
// @retval priority_mask - cached interrupt mask
static inline uint32_t fast__cpu_enter_critical() {
    uint32_t priority_mask = __get_PRIMASK();
    __disable_irq();
    return priority_mask;
}

// @brief Resumes interrupts for critical task completion
// @note These functions enable and disable interrupts (just like their cousins in low_level.h) 
// but must be a static part of this file to be truly inlined
// @param priority_mask - Interrupt mask to restore
static inline void fast__cpu_exit_critical(uint32_t priority_mask) {
    __set_PRIMASK(priority_mask);
}

// @brief Floats ALL phases immediately and disarms both motors and the brake resistor.
static void low_level_fault_fast(Motor::Error_t error) {
    // Disable all motors NOW!
    // Axis 0
    safety_critical_disarm_motor_pwm(axes[0]->motor_);
    axes[0]->motor_.error_ |= error;

    // Axis 1
    #ifndef USE_SINGLE_AXIS
        safety_critical_disarm_motor_pwm(axes[1]->motor_);
        axes[1]->motor_.error_ |= error;
    #endif

    // Disarm brake resistor
    safety_critical_disarm_brake_resistor();
}


// IRQ Callbacks
//----------------------------------------------------------------------------------------------------

// @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
// @ingroup low_level_fast
void ADC_IRQHandler(void) {

    uint16_t now = TIM13->CNT;

    // The HAL's ADC handling mechanism adds many clock cycles of overhead
    // So we bypass it and handle the logic ourselves.
    // ADC1: Injected channel?
    if(__FAST__HAL_ADC_GET_FLAG(ADC1, ADC_FLAG_JEOC)) {

        // Calculate vbus voltage
        vbus_voltage_sense_calculation();

        // Clear interrupt flag & start next conversion
        __FAST__HAL_ADC_MODIFY_FLAG(ADC1, START_INJECTED_CONVERSION);
    }
    
    // ADC2: Injected channel?
    else if(__FAST__HAL_ADC_GET_FLAG(ADC2, ADC_FLAG_JEOC)) {

        pwm_state_machine(true, now);

        // Clear interrupt flag & start next conversion
        __FAST__HAL_ADC_MODIFY_FLAG(ADC2, START_INJECTED_CONVERSION);        
    }

    // ADC2: Regular channel?
    else if(__FAST__HAL_ADC_GET_FLAG(ADC2, ADC_FLAG_EOC)) {

        pwm_state_machine(false, now);

        // Clear interrupt flag & start next conversion
        __FAST__HAL_ADC_MODIFY_FLAG(ADC2, START_REGULAR_CONVERSION);        
    }
}

// @brief This function samples currents from ADC2 and ADC3
// @ingroup low_level_fast
static inline void sample_currents(bool injected, Motor * axis_motor) {
    
    // Storage variables for ADC values 
    uint32_t ADCValue_phB;
    uint32_t ADCValue_phC;

    // Check if this is an injected measurement and if so, get injected values from ADCs
    if (injected) {
        ADCValue_phB = ADC2->JDR1;
        ADCValue_phC = ADC3->JDR1;
    } 

    // Otherwise get regular values from ADCs
    else {
        ADCValue_phB = ADC2->DR;
        ADCValue_phC = ADC3->DR;
    }    

    // Convert motor currents to ADC values
    current_phB_ = axis_motor->phase_current_from_adcval(ADCValue_phB);
    current_phC_ = axis_motor->phase_current_from_adcval(ADCValue_phC);

    //axis_motor->current_meas_adc_samples_[current_measurement_stack_index_][0] = ADCValue_phB;
    //axis_motor->current_meas_adc_samples_[current_measurement_stack_index_][1] = ADCValue_phC;
}

// @brief This is the callback from the ADC that we expect after the PWM has 
// triggered an ADC conversion. It is a low level state machine which keeps track of 
// all the most important step sequencing.
// @ingroup low_level_fast
// @param bool injected - Whether or not the callback is for an injected conversion 
void pwm_state_machine(bool injected, uint16_t now) {
    // Motor 0 is on Timer 1, which triggers ADC2 on Injected, and ADC3 on Injected conversion
    // Motor 1 is on Timer 8, which triggers ADC2 on Regular, and ADC3 on Regular conversion
    bool axis_num = !injected;

    #ifdef USE_SINGLE_AXIS
        Axis& axis = *axes[0];
    #else
        Axis& axis = *axes[axis_num];    
    #endif

    Motor *axis_motor = &axis.motor_;

    // 1 == downcounter
    volatile bool tim1_count_dir = (TIM1->CR1 & TIM_CR1_DIR);

    // Keep track of ADC state & reset when lost sync
    if(injected && (pwm_adc_state_tracker_ >= 12) && (tim1_count_dir == 0)) {

        // If the corresponding timer is counting up, we just sampled in SVM vector 0, i.e. real current
        // If we are counting down, we just sampled in SVM vector 7, with zero current
        //bool counting_down = axis_motor->hw_config_.timer->Instance->CR1 & TIM_CR1_DIR;
        pwm_adc_state_tracker_ = 0;
    }

    // Most important state machine. Check it against this google sheet:
    // https://docs.google.com/spreadsheets/d/1Q7qS4xY7szme8t-sMSE4pxSfSi8XxtwAaxu0Tp91lds/edit#gid=0
    
    // Motor 0 is on Timer 1, which triggers ADC2 on Injected, and ADC3 on Injected conversion
    // Motor 1 is on Timer 8, which triggers ADC2 on Regular, and ADC3 on Regular conversion

    // Setting this flag to true takes a current measurement for the active axis
    bool sample_current_ = false;

    if(injected) {
        // Axis 0 state machine
        switch(pwm_adc_state_tracker_) { 
            // Axis 0 Counting Up  
            case 0:
                // Update control loop for (axis0) on this state
                update_run_control_loop_ = true;        
                // Sample current on this state (axis0, newest [2])
                sample_current_ = true;
                current_measurement_stack_index_ = 2;
                // Set the SVM timings state to (axis0, 1)
                axis0_SVM_timings_state_ = 1;
                // Debugging
                axis_motor->timing_log_[Motor::TIMING_LOG_ADC_CB_DC] = clocks_per_cnt * now;
                break;
            
            // Axis 0 Counting Down
            case 2:
                // Double-sample current on this state (axis0, oldest [0])
                sample_current_ = true;
                current_measurement_stack_index_ = 0;           
                // Do DC bias measurement for (axis0)
                do_dc_calibration_ = true;
                // Set the SVM timings state to (axis0, 2)
                axis0_SVM_timings_state_ = 2;
                break;
            
            // Axis 0 Counting Up
            case 4:
                // Sample current on this state (axis0, oldest [0])
                sample_current_ = true;
                current_measurement_stack_index_ = 0;
                // Set the SVM timings state to (axis0, 3)
                axis0_SVM_timings_state_ = 3;
                break;

            // Axis 0 Counting Down
            case 6:
                // Double-sample current on this state (axis0, middle [1])
                sample_current_ = true;
                current_measurement_stack_index_ = 1;    
                // Set the SVM timings state to (axis0, 4)
                axis0_SVM_timings_state_ = 4;
                break;

            // Axis 0 Counting Up
            case 8:
                // Sample current on this state (axis0, middle [1])        
                sample_current_ = true;
                current_measurement_stack_index_ = 1;  
                // Set the SVM timings state to (axis0, 5)
                axis0_SVM_timings_state_ = 5;
                // Debugging
                axis_motor->timing_log_[Motor::TIMING_LOG_ADC_CB_I] = clocks_per_cnt * now;
                break;

            // Axis 0 Counting Down
            case 10:
                // Double-sample current on this state (axis0, newest [2])
                sample_current_ = true;
                current_measurement_stack_index_ = 2;             
                // Debugging
                check_if_run_control_loop_failed(axis_motor);            
                // Double buffer the axis timings 
                for(int i = 0; i < 6; i++) {
                    axis_motor->active_timings_stack_[i][0] = axis_motor->next_timings_stack_[i][0];
                    axis_motor->active_timings_stack_[i][1] = axis_motor->next_timings_stack_[i][1];
                    axis_motor->active_timings_stack_[i][2] = axis_motor->next_timings_stack_[i][2];
                }
                // Invalidate old timings
                axis_motor->next_timings_valid_ = false;
                // Set the SVM timings state to (axis0, 0)
                // right before the new current loop thread fires         
                axis0_SVM_timings_state_ = 0;
                break;

            default:
                break;
        }
    }

    else {
        // Axis 1 state machine
        switch(pwm_adc_state_tracker_) {        

            // Axis 1 Counting Up
            case 5: 
                // Update control loop for (axis1) on this state
                update_run_control_loop_ = true;         
                // Sample current on this state (axis1, newest [2])        
                sample_current_ = true;            
                current_measurement_stack_index_ = 2;  
                // Set the SVM timings state to (axis1, 1)
                axis1_SVM_timings_state_ = 1;
                break;

            // Axis 1 Counting Down
            case 7: 
                // Double-sample current on this state (axis1, oldest [0])
                sample_current_ = true;
                current_measurement_stack_index_ = 0;               
                // Do DC bias measurement for (axis1)
                do_dc_calibration_ = true;
                // Set the SVM timings state to (axis1, 2)
                axis1_SVM_timings_state_ = 2; 
                break;

            // Axis 1 Counting Up
            case 9:
                // Sample current on this state (axis1, oldest [1])
                sample_current_ = true;
                current_measurement_stack_index_ = 0; 
                // Set the SVM timings state to (axis1, 3)            
                axis1_SVM_timings_state_ = 3;
                // Update brake resistor
                update_brake_current();
                #ifndef USE_SINGLE_AXIS 
                // Debugging
                axis_motor->timing_log_[Motor::TIMING_LOG_ADC_CB_I] = clocks_per_cnt * now;
                #endif     
                break;

            // Axis 1 Counting Down
            case 11:
                // Double-sample current on this state (axis1, middle [1])
                sample_current_ = true;
                current_measurement_stack_index_ = 1;            
                // Set the SVM timings state to (axis1, 4)            
                axis1_SVM_timings_state_ = 4;   
                break;

            // Axis 1 Counting Up    
            case 1:
                // Sample current on this state (axis1, middle [0])
                sample_current_ = true;
                current_measurement_stack_index_ = 1;
                // Set the SVM timings state to (axis1, 5)
                axis1_SVM_timings_state_ = 5;
                #ifndef USE_SINGLE_AXIS 
                // Debugging
                axis_motor->timing_log_[Motor::TIMING_LOG_ADC_CB_DC] = clocks_per_cnt * now;
                #endif
                break;
            
            // Axis 1 Counting Down            
            case 3:
                // Double-sample current on this state (axis1, newest [2])
                sample_current_ = true;
                current_measurement_stack_index_ = 2;       

                #ifndef USE_SINGLE_AXIS 
                // Check whether current thread failed for (axis1)
                check_if_run_control_loop_failed(axis_motor);
                // Double buffer the axis timings 
                for(int i = 0; i < 6; i++) {
                    axis_motor->active_timings_stack_[i][0] = axis_motor->next_timings_stack_[i][0];
                    axis_motor->active_timings_stack_[i][1] = axis_motor->next_timings_stack_[i][1];
                    axis_motor->active_timings_stack_[i][2] = axis_motor->next_timings_stack_[i][2];
                }
                // Invalidate old timings
                axis_motor->next_timings_valid_ = false;
                #endif

                // Set the SVM timings state to (axis1, 0)
                // right before the new current loop thread fires         
                axis1_SVM_timings_state_ = 0;
                break;

            default:
                break;
        }
    }

    // ADC2 and ADC3 record the phB and phC currents concurrently,
    // and their interrupts should arrive on the same clock cycle.
    // We dispatch the callbacks in order, so ADC2 will always be processed before ADC3.
    // Therefore we store the value from ADC2 and signal the thread that the
    // measurement is ready when we receive the ADC3 measurement
    #ifdef USE_SINGLE_AXIS 
        if(axis_num == 0){
    #endif
        sample_currents(injected, axis_motor);
    #ifdef USE_SINGLE_AXIS 
        }
    #endif

    if(do_dc_calibration_) {     
        // Get current samples
        //sample_currents(injected, axis_motor);

        // Take DC bias measurement, and apply low pass filter of about 200ms
        #ifdef USE_SINGLE_AXIS 
            if(axis_num == 0){
        #endif
            axis_motor->DC_calib_.phB += (current_phB_ - axis_motor->DC_calib_.phB) * calib_filter_k;
            axis_motor->DC_calib_.phC += (current_phC_ - axis_motor->DC_calib_.phC) * calib_filter_k;
        #ifdef USE_SINGLE_AXIS 
            }
        #endif

        // Clear flag, nothing left to do!
        do_dc_calibration_ = false;
    }

    // Sample axis currents
    if (sample_current_) {
        // Get current samples
        //sample_currents(injected, axis_motor);

        // Sample current for the active axis
        #ifdef USE_SINGLE_AXIS 
            if(axis_num == 0){
        #endif
            axis_motor->buffered_current_meas_[current_measurement_stack_index_].phB += current_phB_ - axis_motor->DC_calib_.phB;
            axis_motor->buffered_current_meas_[current_measurement_stack_index_].phC += current_phC_ - axis_motor->DC_calib_.phC;
        #ifdef USE_SINGLE_AXIS 
            }
        #endif

        // Clear flag, nothing left to do!
        sample_current_ = false;
    }
    
    // Get sensor samples as early as possible
    if(update_run_control_loop_) {
        // Sample the hall effect sensor pins
        #ifdef USE_SINGLE_AXIS
            if(axis_num == 0) {
        #endif
        for (int i = 0; i < num_GPIO; ++i) {
            GPIO_port_samples[axis_num][i] = GPIOs_to_samp[i]->IDR;
        }
       
        // Sample the encoder
        axis.encoder_.sample_now();

        // Prepare hall readings
        // TODO move this to inside encoder update function
        decode_hall_samples(axis.encoder_, GPIO_port_samples[axis_num]);  
        #ifdef USE_SINGLE_AXIS
            }
        #endif
    }

    // Update PWM timing registers (we do this every state)
    if(axis_num == 0) {
        TIM1->CCR1 = axis_motor->active_timings_stack_[axis0_SVM_timings_state_][0];
        TIM1->CCR2 = axis_motor->active_timings_stack_[axis0_SVM_timings_state_][1];
        TIM1->CCR3 = axis_motor->active_timings_stack_[axis0_SVM_timings_state_][2];
    }
    #ifndef USE_SINGLE_AXIS 
    else {
        TIM8->CCR1 = axis_motor->active_timings_stack_[axis1_SVM_timings_state_][0];
        TIM8->CCR2 = axis_motor->active_timings_stack_[axis1_SVM_timings_state_][1];
        TIM8->CCR3 = axis_motor->active_timings_stack_[axis1_SVM_timings_state_][2];
    }
    #endif

    // Update the current control loop after all the new data has been had
    if(update_run_control_loop_) {

        #ifdef USE_SINGLE_AXIS
            if(axis_num == 0) {
        #endif
        
        // Cache current measurements so they don't get over-written
        for(int i = 0; i < 3; i++) {
            //axis_motor->buffered_current_meas_adc_samples_[i][0] = axis_motor->current_meas_adc_samples_[i][0];
            //axis_motor->buffered_current_meas_adc_samples_[i][1] = axis_motor->current_meas_adc_samples_[i][1];

            axis_motor->current_meas_[i].phB = axis_motor->buffered_current_meas_[i].phB / 2.0f;
            axis_motor->current_meas_[i].phC = axis_motor->buffered_current_meas_[i].phC / 2.0f;

            axis_motor->buffered_current_meas_[i].phB = 0.0f;
            axis_motor->buffered_current_meas_[i].phC = 0.0f;
        }

        // Trigger current control loop update
        axis.signal_current_meas(); 
        
        #ifdef USE_SINGLE_AXIS
            }
        #endif

        // Clear flag, nothing left to do!
        update_run_control_loop_ = false;               
    }

    // Increment the state machine counter
    pwm_adc_state_tracker_ ++;
}


// @brief This function sums up the Ibus contribution of each motor and 
// updates the brake resistor PWM accordingly.
void update_brake_current() {

    // Variable for keeping track of the brake current
    volatile float brake_current = 0.0f;

    // Brake current is the negative current sum from all armed motors
    if (axes[0]->motor_.armed_state_ == Motor::ARMED_STATE_ARMED) {
        brake_current -= axes[0]->motor_.current_control_.Ibus;
    }
    
    #ifndef USE_SINGLE_AXIS
    if (axes[1]->motor_.armed_state_ == Motor::ARMED_STATE_ARMED) {
        brake_current -= axes[1]->motor_.current_control_.Ibus;
    }   
    #endif

    // Clip negative values to 0.0f
    if (brake_current < 0.0f) {
        brake_current = 0.0f;
    }
    
    volatile float Ibus = axes[0]->motor_.current_control_.Ibus;

    // Calculate duty cycle of the brake resistor
    volatile float brake_duty = brake_current * board_config.brake_resistance / vbus_voltage;

    // Duty limit at 90% to allow bootstrap caps to charge
    // If brake_duty is NaN, this expression will also evaluate to false
    if ((brake_duty >= 0.0f) && (brake_duty <= 0.9f)) {
        
        // Calculate duty cycle of the brake resistor
        #ifndef USE_MOTO_PINS
            // Moto uses different brake polarity
            int low_off = static_cast<int>(TIM_APB1_PERIOD_CLOCKS * (brake_duty));
            int high_on = high_on - TIM_APB1_DEADTIME_CLOCKS;
        #else
            // Calculate PWM values from duty cycle
            int high_on = static_cast<int>(TIM_APB1_PERIOD_CLOCKS * (1.0f - brake_duty));
            int low_off = high_on - TIM_APB1_DEADTIME_CLOCKS;
        #endif

        // Constrain bounds of the resistor settings
        if (low_off < 0) { 
            low_off = 0;
        }
        
        // Apply brake resistor timings
        safety_critical_apply_brake_resistor_timings(low_off, high_on);
    } 
    
    else {
        //shuts off all motors AND brake resistor, sets error code on all motors.
        low_level_fault_fast(Motor::ERROR_BRAKE_CURRENT_OUT_OF_RANGE);
    }
}


// @brief This checks if run_control_loop() executed and the SVM timings were  
// updated to a valid new set
void check_if_run_control_loop_failed(Motor * axis_motor) {

    // If the motor is armed, then do a sanity check to see if we are 
    // trying to apply timings that are invalid.
    
    // If the motor control loop went missing, then
    // we must assume that it died and toss an error.
    if(axis_motor->armed_state_ == Motor::ARMED_STATE_ARMED) { 
        
        // Only toss the error if the motor was in an armed state
        //if (axis_motor->next_timings_valid_ == false) {
        if(0) {
            
            // Halt interrupts so task completes without challenge
            uint32_t mask = cpu_enter_critical();

            // Set error flag
            axis_motor->error_ |= Motor::ERROR_CONTROL_DEADLINE_MISSED;
    
            // Set the armed state enum to DISARMED    
            axis_motor->armed_state_ = Motor::ARMED_STATE_DISARMED;

            // Disable timer outputs
            __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(axis_motor->hw_config_.timer);

            // Re-engage interrupts
            cpu_exit_critical(mask);
        }
    }

    // If motor is not armed, step through state machine to arm PWM
    // @todo we really shouldn't be checking this here
    else {
        safety_critical_motor_pwm_timings_state_machine(*axis_motor);
    }
}

// @brief Updates the brake resistor PWM timings unless
// the brake resistor is disarmed.
void safety_critical_apply_brake_resistor_timings(uint32_t low_off, uint32_t high_on) {
    
    // Check if we have a dead-time fault 
    // Disabled for speed... (why would this ever occur except for bad programming?)
    if (high_on - low_off < TIM_APB1_DEADTIME_CLOCKS) {
        low_level_fault_fast(Motor::ERROR_BRAKE_DEADTIME_VIOLATION);
    }
    
    // Disable interuption of this task
    uint32_t mask = fast__cpu_enter_critical();
    
    // Update brake resistor timings if the resistor has been armed
    if (brake_resistor_armed) {
        // Safe update of low and high side timings
        // To avoid race condition, first reset timings to safe state
        // ch3 is low side, ch4 is high side
        TIM2->CCR3 = 0;
        TIM2->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
        TIM2->CCR3 = low_off;
        TIM2->CCR4 = high_on;
    }

    // Re-enable interruption
    fast__cpu_exit_critical(mask);
}

// This function pulls the data from ADC1 injected channel(s) and calculates 
// Vbus Voltage
inline __attribute__((__always_inline__)) void vbus_voltage_sense_calculation() {
        
    // First injected channel is the Vbus Voltage reisistor divider       
    // Scale ADC value to Vbus Voltage
    vbus_voltage = adc_vbus_voltage_scale * (float) ADC1->JDR1;
}