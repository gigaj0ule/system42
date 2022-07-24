/* Includes ------------------------------------------------------------------*/

// Because of broken cmsis_os.h, we need to include arm_math first,
// otherwise chip specific defines are ommited
#include <stm32f405xx.h>
#include <stm32f4xx_hal.h>  // Sets up the correct chip specifc defines required by arm_math
#include "stm32f4xx_it.h"

#define ARM_MATH_CM4
#include <arm_math.h>

#include <cmsis_os.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#include <adc.h>
#include <gpio.h>
#include <main.h>
#include <spi.h>
#include <tim.h>
#include <utils.h>

#include "odrive_main.h"

/* Private defines -----------------------------------------------------------*/

// #define DEBUG_PRINT

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/

// Define constant for calculating Vbus voltage
static constexpr const float adc_full_scale = ADC_FULL_SCALE;
static constexpr const float adc_half_scale = ADC_HALF_SCALE;
static constexpr const float adc_ref_voltage = ADC_REF_VOLTAGE;
static constexpr const float adc_volts_per_count = ADC_VOLTS_PER_COUNT;
static constexpr const float adc_vbus_voltage_scale = ADC_VBUS_VOLTAGE_SCALE;

/* Global variables ----------------------------------------------------------*/
bool brake_resistor_armed = false;

// This is used to track the state of the PWM ADC callback routine
extern int pwm_adc_state_tracker_;
extern bool update_current_control_loop_;

// Init structure for DMA
DMA_HandleTypeDef dma_tim1_ccr_stuffer;

// @brief ADC1 measurements are written to this buffer by DMA
uint16_t adc_measurements_[ADC_CHANNEL_COUNT] = { 0 };


/* Safety critical functions -------------------------------------------------*/

/*
* This section contains all accesses to safety critical hardware registers.
* Specifically, these registers:
*   axes[0]->motor_ PWMs:
*     Timer1.MOE (master output enabled)
*     Timer1.CCR1 (counter compare register 1)
*     Timer1.CCR2 (counter compare register 2)
*     Timer1.CCR3 (counter compare register 3)
*   axes[1]->motor_ PWMs:
*     Timer8.MOE (master output enabled)
*     Timer8.CCR1 (counter compare register 1)
*     Timer8.CCR2 (counter compare register 2)
*     Timer8.CCR3 (counter compare register 3)
*   Brake resistor PWM:
*     Timer2.CCR3 (counter compare register 3)
*     Timer2.CCR4 (counter compare register 4)
* 
* The following assumptions are made:
*   - The hardware operates as described in the datasheet:
*     http://www.st.com/content/ccc/resource/technical/document/reference_manual/3d/6d/5a/66/b4/99/40/d4/DM00031020.pdf/files/DM00031020.pdf/jcr:content/translations/en.DM00031020.pdf
*     This assumption also requires for instance that there are no radiation
*     caused hardware errors.
*   - After startup, all variables used in this section are exclusively modified
*     by the code in this section (this excludes function parameters)
*     This assumption also requires that there is no memory corruption.
*   - This code is compiled by a C standard compliant compiler.
*
* Furthermore:
*   - Between calls to safety_critical_arm_motor_pwm and
*     safety_critical_disarm_motor_pwm the motor's Ibus current is
*     set to the correct value and update_brake_resistor is called
*     at a high rate.
*/


// @brief Floats ALL phases immediately and disarms both motors and the brake resistor.
// @ingroup low_level
void low_level_fault(Motor::Error_t error) {
    // Disable all motors NOW!
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        safety_critical_disarm_motor_pwm(axes[i]->motor_);
        axes[i]->motor_.error_ |= error;
    }

    safety_critical_disarm_brake_resistor();
}

// @brief Kicks off the arming process of the motor.
// All calls to this function must clearly originate
// from user input.
// @ingroup low_level
void safety_critical_arm_motor_pwm(Motor& motor) {
    uint32_t mask = cpu_enter_critical();
    if (brake_resistor_armed) {
        motor.armed_state_ = Motor::ARMED_STATE_WAITING_FOR_TIMINGS;
    }
    cpu_exit_critical(mask);
}

// @brief Disarms the motor PWM.
// After calling this function, it is guaranteed that all three
// motor phases are floating and will not be enabled again until
// safety_critical_arm_motor_phases is called.
// @ingroup low_level
// @returns true if the motor was in a state other than disarmed before
bool safety_critical_disarm_motor_pwm(Motor& motor) {

    // Halt interrupts so task completes without challenge
    uint32_t mask = cpu_enter_critical();
    
    // Check what the previous motor state was
    bool was_armed = motor.armed_state_ != Motor::ARMED_STATE_DISARMED;

    // Invalidate timings
    motor.next_timings_valid_ = false;

    // Set the armed state enum to DISARMED    
    motor.armed_state_ = Motor::ARMED_STATE_DISARMED;

    // Disable timer outputs
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(motor.hw_config_.timer);

    // Re-engage interrupts
    cpu_exit_critical(mask);

    // Return
    return was_armed;
}


// @brief Updates a state machine that keeps track of the motor 
// pwm timings state.
// @ingroup low_level
void safety_critical_motor_pwm_timings_state_machine(Motor& motor) {

    // Halt scheduler to complete this function in its entirety
    uint32_t mask = cpu_enter_critical();

    // If the brake resistor is not engaged we should not run the motor
    if (brake_resistor_armed == false) {
        motor.armed_state_ = Motor::ARMED_STATE_DISARMED;
    }

    // Step through our state machine
    switch (motor.armed_state_)
    {
        case Motor::ARMED_STATE_WAITING_FOR_TIMINGS: {
            // timings were just loaded into the timer registers
            // the timer register are buffered, so they won't have an effect
            // on the output just yet so we need to wait until the next
            // interrupt before we actually enable the output
            motor.armed_state_ = Motor::ARMED_STATE_WAITING_FOR_UPDATE;
            break;
        } 
        case Motor::ARMED_STATE_WAITING_FOR_UPDATE: {
            // now we waited long enough. Enter armed state and
            // enable the actual PWM outputs.
            motor.armed_state_ = Motor::ARMED_STATE_ARMED;

            // enable pwm outputs
            __HAL_TIM_MOE_ENABLE(motor.hw_config_.timer);
            break;
        } 
        case Motor::ARMED_STATE_ARMED: {
            // nothing to do, PWM is running, all good
            break;
        } 
        default: {
            // unknown state oh no
            safety_critical_disarm_motor_pwm(motor);
            break;
        }
    }

    // Release the scheduler
    cpu_exit_critical(mask);
}


// @brief Arms the brake resistor
// @ingroup low_level
void safety_critical_arm_brake_resistor() {
    uint32_t mask = cpu_enter_critical();
    brake_resistor_armed = true;
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    cpu_exit_critical(mask);
}

// @brief Disarms the brake resistor and by extension
// all motor PWM outputs.
// After calling this, the brake resistor can only be armed again
// by calling safety_critical_arm_brake_resistor().
// @ingroup low_level
void safety_critical_disarm_brake_resistor() {
    uint32_t mask = cpu_enter_critical();
    brake_resistor_armed = false;
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        safety_critical_disarm_motor_pwm(axes[i]->motor_);
    }
    cpu_exit_critical(mask);
}


/* Function implementations --------------------------------------------------*/

// @breif Starts ADC and PWM for pwm_state_machine() in low_level_fast.cpp
// @ingroup low_level
void start_adc_pwm() {

    // Enable ADC
    __HAL_ADC_ENABLE(&hadc1);
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_ENABLE(&hadc3);
    
    // Wait for the ADC to enable
    osDelay(2);

    // Enable the ADC interrupts
    __HAL_ADC_ENABLE_IT(&hadc1, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_JEOC);
    __HAL_ADC_ENABLE_IT(&hadc2, ADC_IT_EOC);

    // Ensure that debug halting of the core doesn't leave the motor PWM running
    __HAL_DBGMCU_FREEZE_TIM1();
    __HAL_DBGMCU_FREEZE_TIM8();

    // Start PWM for TIM1 and TIM8
    start_pwm(&htim1);
    start_pwm(&htim8);

    // Synchronize the timers 90 degrees out of phase with each other
    sync_timers(&htim1, &htim8,  TIM_CLOCKSOURCE_ITR0, TIM_1_8_PERIOD_CLOCKS / 2 - 1 * 128, &htim13);

    // Motor output starts in the disabled state
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim1);
    __HAL_TIM_MOE_DISABLE_UNCONDITIONALLY(&htim8);

    // Enable the update interrupt (used to coherently sample GPIO)
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

    // Start brake resistor PWM in floating output configuration
    htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    // Disarm motors and arm brake resistor
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        safety_critical_disarm_motor_pwm(axes[i]->motor_);
    }

    safety_critical_arm_brake_resistor();
}

// @breif Starts the desired PWM timer
// @ingroup low_level
void start_pwm(TIM_HandleTypeDef* htim) {

    // Calculate half-value for the PWM
    int half_load = TIM_1_8_PERIOD_CLOCKS / 2;

    // Preload the timer registers to 50%
    // except for channel 4 which we don't use for PWM
    htim->Instance->CCR1 = half_load;
    htim->Instance->CCR2 = half_load;
    htim->Instance->CCR3 = half_load;
    htim->Instance->CCR4 = 1;

    // Start up all the timers in PWM mode using the HAL
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3);

    // Ecept for channel 4... wait why do we bother with this?
    HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_4);
}

// @brief This function synchronizes TIM1 and TIM8 with a 90 degree offset
// @ingroup low_level
void sync_timers(TIM_HandleTypeDef* htim_a, TIM_HandleTypeDef* htim_b, 
                 uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset,
                 TIM_HandleTypeDef* htim_refbase) {

    // Store intial timer configs
    uint16_t MOE_store_a = htim_a->Instance->BDTR & (TIM_BDTR_MOE);
    uint16_t MOE_store_b = htim_b->Instance->BDTR & (TIM_BDTR_MOE);
    uint16_t CR2_store = htim_a->Instance->CR2;
    uint16_t SMCR_store = htim_b->Instance->SMCR;
    
    // Turn off output
    htim_a->Instance->BDTR &= ~(TIM_BDTR_MOE);
    htim_b->Instance->BDTR &= ~(TIM_BDTR_MOE);
    
    // Disable both timer counters
    htim_a->Instance->CR1 &= ~TIM_CR1_CEN;
    htim_b->Instance->CR1 &= ~TIM_CR1_CEN;

    // Set pwm_adc_state_tracker_ to 0
    pwm_adc_state_tracker_ = 0;
    
    // Set first timer to send TRGO on counter enable
    htim_a->Instance->CR2 &= ~TIM_CR2_MMS;
    htim_a->Instance->CR2 |= TIM_TRGO_ENABLE;
    
    // Set Trigger Source of second timer to the TRGO of the first timer
    htim_b->Instance->SMCR &= ~TIM_SMCR_TS;
    htim_b->Instance->SMCR |= TIM_CLOCKSOURCE_ITRx;
    
    // Set 2nd timer to start on trigger
    htim_b->Instance->SMCR &= ~TIM_SMCR_SMS;
    htim_b->Instance->SMCR |= TIM_SLAVEMODE_TRIGGER;
    
    // Dir bit is read only in center aligned mode, so we clear the mode for now
    uint16_t CMS_store_a = htim_a->Instance->CR1 & TIM_CR1_CMS;
    uint16_t CMS_store_b = htim_b->Instance->CR1 & TIM_CR1_CMS;
    htim_a->Instance->CR1 &= ~TIM_CR1_CMS;
    htim_b->Instance->CR1 &= ~TIM_CR1_CMS;
    
    // Set both timers to up-counting state
    htim_a->Instance->CR1 &= ~TIM_CR1_DIR;
    htim_b->Instance->CR1 &= ~TIM_CR1_DIR;
    
    // Restore center aligned mode
    htim_a->Instance->CR1 |= CMS_store_a;
    htim_b->Instance->CR1 |= CMS_store_b;
    
    // set counter offset
    htim_a->Instance->CNT = count_offset;
    htim_b->Instance->CNT = 0;
    
    // Set and start reference timebase timer (if used)
    if (htim_refbase) {
        htim_refbase->Instance->CNT = count_offset;
        htim_refbase->Instance->CR1 |= (TIM_CR1_CEN); // start
    }

    // Start Timer a
    htim_a->Instance->CR1 |= (TIM_CR1_CEN);
    
    // Restore timer configs
    htim_a->Instance->CR2 = CR2_store;
    htim_b->Instance->SMCR = SMCR_store;

    // restore output
    htim_a->Instance->BDTR |= MOE_store_a;
    htim_b->Instance->BDTR |= MOE_store_b;
}


// @brief Starts the general purpose ADC on the ADC1 peripheral.
// The measured ADC voltages can be read with get_adc_voltage().
//
// ADC1 is set up to continuously sample all channels 0 to 15 in a
// round-robin fashion.
// DMA is used to copy the measured 12-bit values to adc_measurements_.
//
// The injected (high priority) channel of ADC1 is used to sample vbus_voltage.
// This conversion is triggered by TIM1 at the frequency of the motor control loop.
// @ingroup low_level
void start_general_purpose_adc() {

    // Struct for holding ADC initialization values
    ADC_ChannelConfTypeDef sConfig;

    // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = ADC_CHANNEL_COUNT;
    hadc1.Init.DMAContinuousRequests = ENABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;

    // Initialize ADC and catch error if init did not succeed 
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        _Error_Handler((char*)__FILE__, __LINE__);
    }

    // Set up sampling sequence (channel 0 ... channel 15)
    sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
    for (uint32_t channel = 0; channel < ADC_CHANNEL_COUNT; ++channel) {
        sConfig.Channel = channel << ADC_CR1_AWDCH_Pos;
        sConfig.Rank = channel + 1; // rank numbering starts at 1
        if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
            _Error_Handler((char*)__FILE__, __LINE__);
    }

    HAL_ADC_Start_DMA(&hadc1, reinterpret_cast<uint32_t*>(adc_measurements_), ADC_CHANNEL_COUNT);
}



void start_timer_ccr_dma()
{
    // Seems the following are used:
    //hdma_adc1.Instance = DMA2_Stream0;
    //hdma_adc1.Init.Channel = DMA_CHANNEL_0;

    // Just to make sure the clock is on...
    __HAL_RCC_DMA2_CLK_ENABLE();

    // Memory transfer stream
    dma_tim1_ccr_stuffer.Instance = DMA1_Stream7; 

    // Input (FIRE?) signal channel?
    dma_tim1_ccr_stuffer.Init.Channel = DMA_CHANNEL_3; // irrelevant

    // M2P mode 
    dma_tim1_ccr_stuffer.Init.Direction = DMA_MEMORY_TO_PERIPH;

    // Normal DMA transfers
    dma_tim1_ccr_stuffer.Init.Mode = DMA_NORMAL;
    dma_tim1_ccr_stuffer.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    dma_tim1_ccr_stuffer.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;

    // Don't increment address pointers
    dma_tim1_ccr_stuffer.Init.MemInc = DMA_MINC_DISABLE;
    dma_tim1_ccr_stuffer.Init.PeriphInc = DMA_PINC_DISABLE;
    
    // Priority
    dma_tim1_ccr_stuffer.Init.Priority = DMA_PRIORITY_VERY_HIGH;

    // No Fifo
    dma_tim1_ccr_stuffer.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    dma_tim1_ccr_stuffer.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_1QUARTERFULL;

    // @todo lookup what this is
	dma_tim1_ccr_stuffer.Init.MemBurst = DMA_MBURST_SINGLE;
	dma_tim1_ccr_stuffer.Init.PeriphBurst = DMA_PBURST_SINGLE;

    // Intialize the DMA
    if (HAL_DMA_Init(&dma_tim1_ccr_stuffer) != HAL_OK) {
        _Error_Handler((char*)__FILE__, __LINE__);
    }

    // Start DMA
    // Forces transfer transfer now if M2M mode, otherwise transfer waits until peripheral request? 
    //HAL_DMA_Start(&dma_tim1_ccr_stuffer, (uint16_t) axes[0]->motor_.dma_timings_[3], (uint32_t) &htim1.Instance->CCR4, 1);
}


// @brief Returns the ADC voltage associated with the specified pin.
// GPIO_set_to_analog() must be called first to put the Pin into
// analog mode.
// Returns NaN if the pin has no associated ADC1 channel.
//
// On ODrive 3.3 and 3.4 the following pins can be used with this function:
//  GPIO_1, GPIO_2, GPIO_3, GPIO_4 and some pins that are connected to
//  on-board sensors (M0_TEMP, M1_TEMP, AUX_TEMP)
//
// The ADC values are sampled in background at ~30kHz without
// any CPU involvement.
//
// Details: each of the 16 conversion takes (15+26) ADC clock
// cycles and the ADC, so the update rate of the entire sequence is:
//  21000kHz / (15+26) / 16 = 32kHz
// The true frequency is slightly lower because of the injected vbus
// measurements
// @ingroup low_level
// @param GPIO_TypeDef* GPIO_port - STM32 GPIO port ID
// @param uint16_t GPIO_pin - STM32 GPIO pin ID
// @retval float - The voltage sampled in real volts
float get_adc_voltage(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin) {
    uint32_t channel = UINT32_MAX;
    if (GPIO_port == GPIOA) {
        if (GPIO_pin == GPIO_PIN_0)
            channel = 0;
        else if (GPIO_pin == GPIO_PIN_1)
            channel = 1;
        else if (GPIO_pin == GPIO_PIN_2)
            channel = 2;
        else if (GPIO_pin == GPIO_PIN_3)
            channel = 3;
        else if (GPIO_pin == GPIO_PIN_4)
            channel = 4;
        else if (GPIO_pin == GPIO_PIN_5)
            channel = 5;
        else if (GPIO_pin == GPIO_PIN_6)
            channel = 6;
        else if (GPIO_pin == GPIO_PIN_7)
            channel = 7;
    } else if (GPIO_port == GPIOB) {
        if (GPIO_pin == GPIO_PIN_0)
            channel = 8;
        else if (GPIO_pin == GPIO_PIN_1)
            channel = 9;
    } else if (GPIO_port == GPIOC) {
        if (GPIO_pin == GPIO_PIN_0)
            channel = 10;
        else if (GPIO_pin == GPIO_PIN_1)
            channel = 11;
        else if (GPIO_pin == GPIO_PIN_2)
            channel = 12;
        else if (GPIO_pin == GPIO_PIN_3)
            channel = 13;
        else if (GPIO_pin == GPIO_PIN_4)
            channel = 14;
        else if (GPIO_pin == GPIO_PIN_5)
            channel = 15;
    }
    if (channel < ADC_CHANNEL_COUNT)
        return ((float)adc_measurements_[channel]) * (adc_ref_voltage / adc_full_scale);
    else
        return 0.0f / 0.0f; // NaN
}



/* RC PWM input --------------------------------------------------------------*/

// @brief Returns the ODrive GPIO number for a given TIM2 or TIM5 input capture channel number.
// @ingroup low_level
// @param int channel - The input capture channel number to process
int tim_2_5_channel_num_to_gpio_num(int channel) {
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    if (channel >= 1 && channel <= 4) {
        // the channel numbers just happen to coincide with
        // the GPIO numbers
        return channel;
    } 
    
    else {
        return -1;
    }
#else
    // Only ch4 is available on v3.2
    if (channel == 4) {
        return 4;
    } 
    else {
        return -1;
    }
#endif
}

// @brief Returns the TIM2 or TIM5 channel number for a given GPIO number.
// @ingroup low_level
// @param int gpio_num - The gpio number to process
uint32_t gpio_num_to_tim_2_5_channel(int gpio_num) {

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    switch (gpio_num) {
        case 1: return TIM_CHANNEL_1;
        case 2: return TIM_CHANNEL_2;
        case 3: return TIM_CHANNEL_3;
        case 4: return TIM_CHANNEL_4;
        default: return 0;
    }
#else
    // Only ch4 is available on v3.2
    if (gpio_num == 4) {
        return TIM_CHANNEL_4;
    } 
    else {
        return 0;
    }
#endif
}

// @brief This function starts the PWM input capture routine
// @ingroup low_level
void start_pwm_input_capture() {
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;

    TIM_IC_InitTypeDef sConfigIC;
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 15;

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 3
    for (int gpio_num = 1; gpio_num <= 4; ++gpio_num) {
#else
    int gpio_num = 4; {
#endif
        if (is_endpoint_ref_valid(board_config.pwm_mappings[gpio_num - 1].endpoint)) {
            GPIO_InitStruct.Pin = get_gpio_pin_by_pin(gpio_num);
            HAL_GPIO_DeInit(get_gpio_port_by_pin(gpio_num), get_gpio_pin_by_pin(gpio_num));
            HAL_GPIO_Init(get_gpio_port_by_pin(gpio_num), &GPIO_InitStruct);
            HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, gpio_num_to_tim_2_5_channel(gpio_num));
            HAL_TIM_IC_Start_IT(&htim5, gpio_num_to_tim_2_5_channel(gpio_num));
        }
    }
}

//TODO: These expressions have integer division by 1MHz, so it will be incorrect for clock speeds of not-integer MHz
#define TIM_2_5_CLOCK_HZ        TIM_APB1_CLOCK_HZ
#define PWM_MIN_HIGH_TIME          ((TIM_2_5_CLOCK_HZ / 1000000UL) * 1000UL) // 1ms high is considered full reverse
#define PWM_MAX_HIGH_TIME          ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2000UL) // 2ms high is considered full forward
#define PWM_MIN_LEGAL_HIGH_TIME    ((TIM_2_5_CLOCK_HZ / 1000000UL) * 500UL) // ignore high periods shorter than 0.5ms
#define PWM_MAX_LEGAL_HIGH_TIME    ((TIM_2_5_CLOCK_HZ / 1000000UL) * 2500UL) // ignore high periods longer than 2.5ms
#define PWM_INVERT_INPUT        false


bool is_endpoint_ref_valid(endpoint_ref_t endpoint_ref) {
    return (endpoint_ref.json_crc == json_crc_)
        && (endpoint_ref.endpoint_id < n_endpoints_);
}

Endpoint* get_endpoint(endpoint_ref_t endpoint_ref) {
    if (is_endpoint_ref_valid(endpoint_ref))
        return endpoint_list_[endpoint_ref.endpoint_id];
    else
        return nullptr;
}

// @brief This function handles a PWM pulse and does something with it
// @ingroup low_level_fast
void handle_pulse(int gpio_num, uint32_t high_time) {
    if (high_time < PWM_MIN_LEGAL_HIGH_TIME || high_time > PWM_MAX_LEGAL_HIGH_TIME)
        return;

    if (high_time < PWM_MIN_HIGH_TIME)
        high_time = PWM_MIN_HIGH_TIME;
    if (high_time > PWM_MAX_HIGH_TIME)
        high_time = PWM_MAX_HIGH_TIME;
    float fraction = (float)(high_time - PWM_MIN_HIGH_TIME) / (float)(PWM_MAX_HIGH_TIME - PWM_MIN_HIGH_TIME);
    float value = board_config.pwm_mappings[gpio_num - 1].min +
                  (fraction * (board_config.pwm_mappings[gpio_num - 1].max - board_config.pwm_mappings[gpio_num - 1].min));

    Endpoint* endpoint = get_endpoint(board_config.pwm_mappings[gpio_num - 1].endpoint);
    if (!endpoint)
        return;

    endpoint->set_from_float(value);
}

// @brief This function handles callbacks from the input capture system
// @ingroup low_level
void pwm_input_capture_callback(int channel, uint32_t timestamp) {
    static uint32_t last_timestamp[GPIO_COUNT] = { 0 };
    static bool last_pin_state[GPIO_COUNT] = { false };
    static bool last_sample_valid[GPIO_COUNT] = { false };

    int gpio_num = tim_2_5_channel_num_to_gpio_num(channel);
    if (gpio_num < 1 || gpio_num > GPIO_COUNT)
        return;
    bool current_pin_state = HAL_GPIO_ReadPin(get_gpio_port_by_pin(gpio_num), get_gpio_pin_by_pin(gpio_num)) != GPIO_PIN_RESET;

    if (last_sample_valid[gpio_num - 1]
        && (last_pin_state[gpio_num - 1] != PWM_INVERT_INPUT)
        && (current_pin_state == PWM_INVERT_INPUT)) {
        handle_pulse(gpio_num, timestamp - last_timestamp[gpio_num - 1]);
    }

    last_timestamp[gpio_num - 1] = timestamp;
    last_pin_state[gpio_num - 1] = current_pin_state;
    last_sample_valid[gpio_num - 1] = true;
}


/* Analog speed control input */

// @brief This function handles the analog from communication subsystem
// @ingroup low_level
static void update_analog_endpoint(const struct PWMMapping_t *map, int gpio) {
    float fraction = get_adc_voltage(get_gpio_port_by_pin(gpio), get_gpio_pin_by_pin(gpio)) / 3.3f;
    float value = map->min + (fraction * (map->max - map->min));
    get_endpoint(map->endpoint)->set_from_float(value);
}

// @brief This function samples continuously from analog GPIO
// @ingroup low_level
static void analog_polling_thread(void *) {
    while (true) {
        for (int i = 0; i < GPIO_COUNT; i++) {
            struct PWMMapping_t *map = &board_config.analog_mappings[i];

            if (is_endpoint_ref_valid(map->endpoint)) {
                update_analog_endpoint(map, i + 1);
            }
        }
        osDelay(10);
    }
}

// @brief This function starts analog_polling_thread()
// @ingroup low_level
void start_analog_thread() {
    osThreadDef(analog_thread, analog_polling_thread, osPriorityNormal, 0, 3 * 512);
    osThreadCreate(osThread(analog_thread), NULL);
}
