#include <Arduino.h>
#include <interrupt.h>

#include "dsp_functions.hpp"

#ifdef __STM32F1__
    #error "No F1 support yet in dsp_functions.cpp"
#endif

#ifndef __STM32F4__
    #define __STM32F4__
#endif

#define USE_SINGLE_AXIS

#ifdef __STM32F1__
    #include <stm32f1xx_hal_gpio.h>
    #include <stm32f1xx_hal_adc.h>
    #include <stm32f1xx_hal_rcc.h>
    #include <stm32f1xx_hal_tim.h>
#endif

#ifdef __STM32F4__
    #include <stm32f405xx.h>
    #include <stm32f4xx_hal_gpio.h>
    #include <stm32f4xx_hal_gpio_ex.h>
    #include <stm32f4xx_hal_adc.h>
    #include <stm32f4xx_hal_adc_ex.h>
    #include <stm32f4xx_hal_rcc.h>
    #include <stm32f4xx_hal_rcc_ex.h>
    #include <stm32f4xx_hal_tim.h>
    #include <stm32f4xx_hal_tim_ex.h>

    //#include "stm32f4xx.h"
    //#include "stm32f4xx_it.h"
#endif

// ---------------------------------------------------------------------------
// Prototypes
// 
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle);


// ---------------------------------------------------------------------------
// Methods
//
void set_pwm(TIM_HandleTypeDef * htim, int a, int b, int c) {
}


// Timer initialization ------------------------------------------------------
//

// To trigger the ADC, we must use an output channel that is in PWM mode
// However, CubeMX does not allow you to set up a channel as PWM without an output pin.
// This will set OC4 to PWM mode. Also, triggering doesn't work if the compare register
// (called pulse here) is 0, so we initialise it to 1.
void OC4_PWM_Override(TIM_HandleTypeDef* htim) {

    TIM_OC_InitTypeDef      sConfigOC;
    sConfigOC.OCMode        = TIM_OCMODE_PWM2;
    sConfigOC.Pulse         = 1;
    sConfigOC.OCPolarity    = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity   = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode    = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState   = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState  = TIM_OCNIDLESTATE_RESET;

    HAL_TIM_OC_ConfigChannel(htim, &sConfigOC, TIM_CHANNEL_4);
}

// ---------------------------------------------------------------------------
// TIM1 Init
//
TIM_HandleTypeDef htim1;

void DSP_TIM1_Init(void) {

    // Initialize the time base
    htim1.Instance                  = TIM1;
    htim1.Init.Prescaler            = 0;
    htim1.Init.CounterMode          = TIM_COUNTERMODE_CENTERALIGNED3;
    htim1.Init.Period               = TIM_1_8_PERIOD_CLOCKS;
    htim1.Init.ClockDivision        = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter    = TIM_1_8_RCR;

    // Commit the time base config
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Initialize clock source
    TIM_ClockConfigTypeDef          sClockSourceConfig;
    sClockSourceConfig.ClockSource  = TIM_CLOCKSOURCE_INTERNAL;

    // Commit clock config
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Commit PWM
    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Commit OC
    if (HAL_TIM_OC_Init(&htim1) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Initialize triggers
    TIM_MasterConfigTypeDef             sMasterConfig;
    sMasterConfig.MasterOutputTrigger   = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode       = TIM_MASTERSLAVEMODE_DISABLE;

    // Write triggers
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Output Compare configuration
    TIM_OC_InitTypeDef                  sConfigOC;
    sConfigOC.OCMode                    = TIM_OCMODE_PWM2;
    sConfigOC.Pulse                     = 0;
    sConfigOC.OCPolarity                = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity               = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode                = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState               = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState              = TIM_OCNIDLESTATE_RESET;

    // Init CH1
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Init CH2
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Init CH3
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // CH4 is not used
    sConfigOC.OCMode                    = TIM_OCMODE_TIMING;

    // Init CH4
    if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configure dead time
    TIM_BreakDeadTimeConfigTypeDef          sBreakDeadTimeConfig;
    sBreakDeadTimeConfig.OffStateRunMode    = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode   = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel          = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime           = TIM_1_8_DEADTIME_CLOCKS;
    sBreakDeadTimeConfig.BreakState         = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity      = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput    = TIM_AUTOMATICOUTPUT_DISABLE;

    // Commit dead time
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Init pins
    //HAL_TIM_MspPostInit(&htim1);
}

// ---------------------------------------------------------------------------
// TIM8 Init
//
TIM_HandleTypeDef htim8;

void DSP_TIM8_Init(void) {

    // Initialize the time base
    htim8.Instance                      = TIM8;
    htim8.Init.Prescaler                = 0;
    htim8.Init.CounterMode              = TIM_COUNTERMODE_CENTERALIGNED3;
    htim8.Init.Period                   = TIM_1_8_PERIOD_CLOCKS;
    htim8.Init.ClockDivision            = TIM_CLOCKDIVISION_DIV1;
    htim8.Init.RepetitionCounter        = TIM_1_8_RCR;

    // Commit the time base config
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Trigger the timer from TRGO
    TIM_MasterConfigTypeDef             sMasterConfig;
    sMasterConfig.MasterOutputTrigger   = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode       = TIM_MASTERSLAVEMODE_DISABLE;

    // Commit trigger settings
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Ouptut compare settings
    TIM_OC_InitTypeDef                  sConfigOC;
    sConfigOC.OCMode                    = TIM_OCMODE_PWM2;
    sConfigOC.Pulse                     = 0;
    sConfigOC.OCPolarity                = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity               = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode                = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState               = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState              = TIM_OCNIDLESTATE_RESET;

    // Commit PWM settings for CH1
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Commit PWM settings for CH2
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Commit PWM settings for CH3
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configure dead time
    TIM_BreakDeadTimeConfigTypeDef          sBreakDeadTimeConfig;
    sBreakDeadTimeConfig.OffStateRunMode    = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode   = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel          = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime           = TIM_1_8_DEADTIME_CLOCKS;
    sBreakDeadTimeConfig.BreakState         = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity      = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput    = TIM_AUTOMATICOUTPUT_DISABLE;

    // Commit dead time
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Init pins
    HAL_TIM_MspPostInit(&htim8);
}

// ---------------------------------------------------------------------------
// TIM3 Init
//
TIM_HandleTypeDef htim3;

void DSP_TIM3_Init(void) {

    // Configure timer
    htim3.Instance              = TIM3;
    htim3.Init.Prescaler        = 0;
    htim3.Init.CounterMode      = TIM_COUNTERMODE_UP;
    htim3.Init.Period           = 0xffff;
    htim3.Init.ClockDivision    = TIM_CLOCKDIVISION_DIV1;

    // Configure timer for ecnoder
    TIM_Encoder_InitTypeDef     sConfig;
    sConfig.EncoderMode         = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity         = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection        = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler        = TIM_ICPSC_DIV1;
    sConfig.IC1Filter           = 4;
    sConfig.IC2Polarity         = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection        = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler        = TIM_ICPSC_DIV1;
    sConfig.IC2Filter           = 4;

    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configure triggers
    TIM_MasterConfigTypeDef             sMasterConfig;
    sMasterConfig.MasterOutputTrigger   = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode       = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }
}

// ---------------------------------------------------------------------------
// TIM4 Init
//
TIM_HandleTypeDef htim4;

void DSP_TIM4_Init(void) {

    #ifndef USE_SINGLE_AXIS
        // Configure timer
        htim4.Instance              = TIM4;
        htim4.Init.Prescaler        = 0;
        htim4.Init.CounterMode      = TIM_COUNTERMODE_UP;
        htim4.Init.Period           = 0xffff;
        htim4.Init.ClockDivision    = TIM_CLOCKDIVISION_DIV1;

        // Configure timer for ecnoder
        TIM_Encoder_InitTypeDef     sConfig;
        sConfig.EncoderMode         = TIM_ENCODERMODE_TI12;
        sConfig.IC1Polarity         = TIM_ICPOLARITY_RISING;
        sConfig.IC1Selection        = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC1Prescaler        = TIM_ICPSC_DIV1;
        sConfig.IC1Filter           = 4;
        sConfig.IC2Polarity         = TIM_ICPOLARITY_RISING;
        sConfig.IC2Selection        = TIM_ICSELECTION_DIRECTTI;
        sConfig.IC2Prescaler        = TIM_ICPSC_DIV1;
        sConfig.IC2Filter           = 4;

        if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
            _Error_Handler(__FILE__, __LINE__);
        }

        // Configure triggers
        TIM_MasterConfigTypeDef             sMasterConfig;
        sMasterConfig.MasterOutputTrigger   = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode       = TIM_MASTERSLAVEMODE_DISABLE;

        if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
            _Error_Handler(__FILE__, __LINE__);
        }
    #endif
}

// ---------------------------------------------------------------------------
// TIM2 Init
//
TIM_HandleTypeDef htim2;

void DSP_TIM2_Init(void) {

    // Configure timer
    htim2.Instance            = TIM2;
    htim2.Init.Prescaler      = 0;
    htim2.Init.CounterMode    = TIM_COUNTERMODE_CENTERALIGNED3;
    htim2.Init.Period         = TIM_APB1_PERIOD_CLOCKS;
    htim2.Init.ClockDivision  = TIM_CLOCKDIVISION_DIV1;

    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configure timer for ecnoder
    TIM_MasterConfigTypeDef               sMasterConfig;
    sMasterConfig.MasterOutputTrigger     = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode         = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configure output compare
    TIM_OC_InitTypeDef    sConfigOC;
    sConfigOC.OCMode      = TIM_OCMODE_PWM2;
    sConfigOC.Pulse       = 0;
    sConfigOC.OCPolarity  = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode  = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configure triggers
    sConfigOC.Pulse         = TIM_APB1_PERIOD_CLOCKS+1;
    sConfigOC.OCPolarity    = TIM_OCPOLARITY_HIGH;

    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Init GPIO
    HAL_TIM_MspPostInit(&htim2);
}

// ---------------------------------------------------------------------------
// TIM5 Init
//
TIM_HandleTypeDef htim5;

void DSP_TIM5_Init(void) {

    // Configure timer
    htim5.Instance              = TIM5;
    htim5.Init.Prescaler        = 0;
    htim5.Init.CounterMode      = TIM_COUNTERMODE_UP;
    htim5.Init.Period           = 0xFFFFFFFF;
    htim5.Init.ClockDivision    = TIM_CLOCKDIVISION_DIV1;

    if (HAL_TIM_IC_Init(&htim5) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Config triggers
    TIM_MasterConfigTypeDef             sMasterConfig;
    sMasterConfig.MasterOutputTrigger   = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode       = TIM_MASTERSLAVEMODE_DISABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Config input capture
    TIM_IC_InitTypeDef      sConfigIC;
    sConfigIC.ICPolarity    = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    sConfigIC.ICSelection   = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler   = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter      = 15;

    if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }
}

// ---------------------------------------------------------------------------
// TIM13 Init
//
TIM_HandleTypeDef htim13;

void DSP_TIM13_Init(void) {

    // Configure timer
    htim13.Instance             = TIM13;
    htim13.Init.Prescaler       = 0;
    htim13.Init.CounterMode     = TIM_COUNTERMODE_UP;
    htim13.Init.Period          = (2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR+1)) * ((float)TIM_APB1_CLOCK_HZ / (float)TIM_1_8_CLOCK_HZ) - 1;
    htim13.Init.ClockDivision   = TIM_CLOCKDIVISION_DIV1;

    if (HAL_TIM_Base_Init(&htim13) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }
}

// ---------------------------------------------------------------------------
//
//
void DSP_DMA_Init(void) {

    // DMA controller clock enable
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    // DMA interrupt init
    // DMA1_Stream0_IRQn interrupt configuration
    HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
    
    // DMA1_Stream2_IRQn interrupt configuration
    HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
    
    // DMA1_Stream4_IRQn interrupt configuration
    HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
    
    // DMA1_Stream5_IRQn interrupt configuration
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
}

// ---------------------------------------------------------------------------
// HAL_TIM_Base_MspInit (already definied by Arduino.h)
//
#ifdef INC_MSPINIT
    void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle) {

        if(tim_baseHandle->Instance==TIM1) {
            __HAL_RCC_TIM1_CLK_ENABLE();
        }

        else if(tim_baseHandle->Instance==TIM13) {
            __HAL_RCC_TIM13_CLK_ENABLE();
        }
    }
#endif

// ---------------------------------------------------------------------------
// HAL_TIM_Base_MspDeInit (already definied by Arduino.h)
//
#ifdef INC_MSPINIT
    void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle) {

        if(tim_baseHandle->Instance==TIM1) {
            __HAL_RCC_TIM1_CLK_DISABLE();
            HAL_NVIC_DisableIRQ(TIM1_UP_TIM10_IRQn);
        }

        else if(tim_baseHandle->Instance==TIM13) {
            __HAL_RCC_TIM13_CLK_DISABLE();
        }
    }
#endif

// ---------------------------------------------------------------------------
// HAL_TIM_IC_MspInit (already definied by Arduino.h)
//
#ifdef INC_MSPINIT
    void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle) {

        GPIO_InitTypeDef GPIO_InitStruct;

        if(tim_icHandle->Instance==TIM5) {
            __HAL_RCC_TIM5_CLK_ENABLE();

            // PA2     ------> TIM5_CH3
            // PA3     ------> TIM5_CH4

            GPIO_InitStruct.Pin         = GPIO_3_Pin|GPIO_4_Pin;
            GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Pull        = GPIO_NOPULL;
            GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_LOW;
            GPIO_InitStruct.Alternate   = GPIO_AF2_TIM5;
            HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

            HAL_NVIC_SetPriority(TIM5_IRQn, 5, 0);
            HAL_NVIC_EnableIRQ(TIM5_IRQn);
        }
    }
#endif

// ---------------------------------------------------------------------------
// HAL_TIM_IC_MspDeInit (already definied by Arduino.h)
//
#ifdef INC_MSPINIT
    void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* tim_icHandle) {

        if(tim_icHandle->Instance==TIM5) {
            __HAL_RCC_TIM5_CLK_DISABLE();

            // PA2     ------> TIM5_CH3
            // PA3     ------> TIM5_CH4
            HAL_GPIO_DeInit(GPIOA, GPIO_3_Pin|GPIO_4_Pin);

            HAL_NVIC_DisableIRQ(TIM5_IRQn);
        }
    }
#endif

// ---------------------------------------------------------------------------
//
//
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle) {

    if(tim_pwmHandle->Instance==TIM2) {
        __HAL_RCC_TIM2_CLK_ENABLE();
    }

    else if(tim_pwmHandle->Instance==TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
        //HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, 0, 0);
        //HAL_NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);
    }
}

// ---------------------------------------------------------------------------
//
//
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* tim_encoderHandle) {

    GPIO_InitTypeDef GPIO_InitStruct;

    if(tim_encoderHandle->Instance==TIM3) {
        __HAL_RCC_TIM3_CLK_ENABLE();

        // PB4     ------> TIM3_CH1
        // PB5     ------> TIM3_CH2

        GPIO_InitStruct.Pin         = M0_ENC_A_Pin|M0_ENC_B_Pin;
        GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull        = GPIO_NOPULL;
        GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate   = GPIO_AF2_TIM3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }

    else if(tim_encoderHandle->Instance==TIM4) {
        
        #ifndef USE_SINGLE_AXIS
            __HAL_RCC_TIM4_CLK_ENABLE();

            // PB6     ------> TIM4_CH1
            // PB7     ------> TIM4_CH2

            GPIO_InitStruct.Pin         = M1_ENC_A_Pin|M1_ENC_B_Pin;
            GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Pull        = GPIO_NOPULL;
            GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_LOW;
            GPIO_InitStruct.Alternate   = GPIO_AF2_TIM4;
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        #endif
    }
}

// ---------------------------------------------------------------------------
//
//
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle) {

    GPIO_InitTypeDef GPIO_InitStruct;

    if(timHandle->Instance==TIM1) {

        // PB13     ------> TIM1_CH1N
        // PB14     ------> TIM1_CH2N
        // PB15     ------> TIM1_CH3N
        // PA8      ------> TIM1_CH1
        // PA9      ------> TIM1_CH2
        // PA10     ------> TIM1_CH3

        // LOW PINS
        GPIO_InitStruct.Pin         = M0_AL_Pin|M0_BL_Pin|M0_CL_Pin;
        GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull        = GPIO_NOPULL;
        GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate   = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        // HIGH PINS
        GPIO_InitStruct.Pin         = M0_AH_Pin|M0_BH_Pin|M0_CH_Pin;
        GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull        = GPIO_NOPULL;
        GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate   = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }

    // TIM2 -----------------------------
    else if(timHandle->Instance==TIM2) {

        // PB10     ------> TIM2_CH3
        // PB11     ------> TIM2_CH4

        GPIO_InitStruct.Pin         = AUX_L_Pin|AUX_H_Pin;
        GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull        = GPIO_NOPULL;
        GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate   = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }

    // TIM8 -----------------------------
    else if(timHandle->Instance==TIM8) {
    
        #ifndef USE_SINGLE_AXIS
            // PA7     ------> TIM8_CH1N
            // PB0     ------> TIM8_CH2N
            // PB1     ------> TIM8_CH3N
            // PC6     ------> TIM8_CH1
            // PC7     ------> TIM8_CH2
            // PC8     ------> TIM8_CH3 
        
            GPIO_InitStruct.Pin         = M1_AL_Pin;
            GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Pull        = GPIO_NOPULL;
            GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_LOW;
            GPIO_InitStruct.Alternate   = GPIO_AF3_TIM8;
            HAL_GPIO_Init(M1_AL_GPIO_Port, &GPIO_InitStruct);

            GPIO_InitStruct.Pin         = M1_BL_Pin|M1_CL_Pin;
            GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Pull        = GPIO_NOPULL;
            GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_LOW;
            GPIO_InitStruct.Alternate   = GPIO_AF3_TIM8;
            HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

            GPIO_InitStruct.Pin         = M1_AH_Pin|M1_BH_Pin|M1_CH_Pin;
            GPIO_InitStruct.Mode        = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Pull        = GPIO_NOPULL;
            GPIO_InitStruct.Speed       = GPIO_SPEED_FREQ_LOW;
            GPIO_InitStruct.Alternate   = GPIO_AF3_TIM8;
            HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
        #endif
    }
}

// ---------------------------------------------------------------------------
//
//
void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle) {

    if(tim_pwmHandle->Instance==TIM2) {
        __HAL_RCC_TIM2_CLK_DISABLE();
    }

    else if(tim_pwmHandle->Instance==TIM8) {
        __HAL_RCC_TIM8_CLK_DISABLE();

        HAL_NVIC_DisableIRQ(TIM8_TRG_COM_TIM14_IRQn);
    }
}

// ---------------------------------------------------------------------------
//
//
void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* tim_encoderHandle) {

    if(tim_encoderHandle->Instance==TIM3) {
        __HAL_RCC_TIM3_CLK_DISABLE();

        // PB4     ------> TIM3_CH1
        // PB5     ------> TIM3_CH2

        HAL_GPIO_DeInit(GPIOB, M0_ENC_A_Pin|M0_ENC_B_Pin);
    }

    else if(tim_encoderHandle->Instance==TIM4) {

    #ifndef USE_SINGLE_AXIS
        __HAL_RCC_TIM4_CLK_DISABLE();
        // PB6     ------> TIM4_CH1
        // PB7     ------> TIM4_CH2
        HAL_GPIO_DeInit(GPIOB, M1_ENC_A_Pin|M1_ENC_B_Pin);
    #endif
    }
}


// ---------------------------------------------------------------------------
// ADC1 Init
//
ADC_HandleTypeDef hadc1;

void DSP_ADC1_Init(void) {

    ADC_ChannelConfTypeDef sConfig;
    ADC_InjectionConfTypeDef sConfigInjected;

    // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    hadc1.Instance                      = ADC1;
    hadc1.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution               = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode             = DISABLE;
    hadc1.Init.ContinuousConvMode       = DISABLE;
    hadc1.Init.DiscontinuousConvMode    = DISABLE;
    hadc1.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv         = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion          = 1;
    hadc1.Init.DMAContinuousRequests    = DISABLE;
    hadc1.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    sConfig.Channel         = ADC_CHANNEL_6;
    sConfig.Rank            = 1;
    sConfig.SamplingTime    = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    sConfigInjected.InjectedChannel                 = ADC_CHANNEL_6;
    sConfigInjected.InjectedRank                    = 1;
    sConfigInjected.InjectedNbrOfConversion         = 1;
    sConfigInjected.InjectedSamplingTime            = ADC_SAMPLETIME_3CYCLES;
    sConfigInjected.ExternalTrigInjecConvEdge       = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
    sConfigInjected.ExternalTrigInjecConv           = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
    sConfigInjected.AutoInjectedConv                = DISABLE;
    sConfigInjected.InjectedDiscontinuousConvMode   = DISABLE;
    sConfigInjected.InjectedOffset                  = 0;

    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

// ---------------------------------------------------------------------------
// ADC2 Init
//
ADC_HandleTypeDef hadc2;

void DSP_ADC2_Init(void) {
    
    ADC_ChannelConfTypeDef sConfig;
    ADC_InjectionConfTypeDef sConfigInjected;

    // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    hadc2.Instance                      = ADC2;
    hadc2.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc2.Init.Resolution               = ADC_RESOLUTION_12B;
    hadc2.Init.ScanConvMode             = DISABLE;
    hadc2.Init.ContinuousConvMode       = DISABLE;
    hadc2.Init.DiscontinuousConvMode    = DISABLE;
    hadc2.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc2.Init.ExternalTrigConv         = ADC_EXTERNALTRIGCONV_T8_TRGO;
    hadc2.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion          = 1;
    hadc2.Init.DMAContinuousRequests    = DISABLE;
    hadc2.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    sConfig.Channel                     = ADC_CHANNEL_13;
    sConfig.Rank                        = 1;
    sConfig.SamplingTime                = ADC_SAMPLETIME_3CYCLES;

    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    sConfigInjected.InjectedChannel                 = ADC_CHANNEL_10;
    sConfigInjected.InjectedRank                    = 1;
    sConfigInjected.InjectedNbrOfConversion         = 1;
    sConfigInjected.InjectedSamplingTime            = ADC_SAMPLETIME_3CYCLES;
    sConfigInjected.ExternalTrigInjecConvEdge       = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
    sConfigInjected.ExternalTrigInjecConv           = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
    sConfigInjected.AutoInjectedConv                = DISABLE;
    sConfigInjected.InjectedDiscontinuousConvMode   = DISABLE;
    sConfigInjected.InjectedOffset                  = 0;

    if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }
}

// ---------------------------------------------------------------------------
// ADC3 Init
//
ADC_HandleTypeDef hadc3;

void DSP_ADC3_Init(void) {

    ADC_ChannelConfTypeDef sConfig;
    ADC_InjectionConfTypeDef sConfigInjected;

    // Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    hadc3.Instance                      = ADC3;
    hadc3.Init.ClockPrescaler           = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc3.Init.Resolution               = ADC_RESOLUTION_12B;
    hadc3.Init.ScanConvMode             = DISABLE;
    hadc3.Init.ContinuousConvMode       = DISABLE;
    hadc3.Init.DiscontinuousConvMode    = DISABLE;
    hadc3.Init.ExternalTrigConvEdge     = ADC_EXTERNALTRIGCONVEDGE_RISING;
    hadc3.Init.ExternalTrigConv         = ADC_EXTERNALTRIGCONV_T8_TRGO;
    hadc3.Init.DataAlign                = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion          = 1;
    hadc3.Init.DMAContinuousRequests    = DISABLE;
    hadc3.Init.EOCSelection             = ADC_EOC_SINGLE_CONV;

    if (HAL_ADC_Init(&hadc3) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    sConfig.Channel         = ADC_CHANNEL_12;
    sConfig.Rank            = 1;
    sConfig.SamplingTime    = ADC_SAMPLETIME_3CYCLES;
    
    if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    // Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time 
    sConfigInjected.InjectedChannel                 = ADC_CHANNEL_11;
    sConfigInjected.InjectedRank                    = 1;
    sConfigInjected.InjectedNbrOfConversion         = 1;
    sConfigInjected.InjectedSamplingTime            = ADC_SAMPLETIME_3CYCLES;
    sConfigInjected.ExternalTrigInjecConvEdge       = ADC_EXTERNALTRIGINJECCONVEDGE_RISING;
    sConfigInjected.ExternalTrigInjecConv           = ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
    sConfigInjected.AutoInjectedConv                = DISABLE;
    sConfigInjected.InjectedDiscontinuousConvMode   = DISABLE;
    sConfigInjected.InjectedOffset                  = 0;

    if (HAL_ADCEx_InjectedConfigChannel(&hadc3, &sConfigInjected) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }
}

// ---------------------------------------------------------------------------
// HAL_ADC_MspInit (already definied by Arduino.h)
//
DMA_HandleTypeDef hdma_adc1;

//void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle) {
void DSP_HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle) {

    GPIO_InitTypeDef GPIO_InitStruct;

    if(adcHandle->Instance==ADC1) {
        __HAL_RCC_ADC1_CLK_ENABLE();
    
        // ADC1 GPIO Configuration    
        // PC0     ------> ADC1_IN10
        // PC1     ------> ADC1_IN11
        // PC2     ------> ADC1_IN12
        // PC3     ------> ADC1_IN13
        // PA4     ------> ADC1_IN4
        // PA5     ------> ADC1_IN5
        // PA6     ------> ADC1_IN6
        // PC5     ------> ADC1_IN15 

        #ifndef USE_SINGLE_AXIS
            GPIO_InitStruct.Pin   = M0_IB_Pin|M0_IC_Pin|M1_IC_Pin|M1_IB_Pin 
                                |M0_TEMP_Pin;
        #else
            GPIO_InitStruct.Pin   = M0_IB_Pin|M0_IC_Pin|M0_TEMP_Pin;
        #endif

        GPIO_InitStruct.Mode    = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull    = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        #ifndef USE_SINGLE_AXIS
            GPIO_InitStruct.Pin   = M1_TEMP_Pin|AUX_TEMP_Pin|VBUS_S_Pin;
        #else
            GPIO_InitStruct.Pin   = AUX_TEMP_Pin|VBUS_S_Pin;
        #endif
        
        GPIO_InitStruct.Mode    = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull    = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        hdma_adc1.Instance                  = DMA2_Stream0;
        hdma_adc1.Init.Channel              = DMA_CHANNEL_0;
        hdma_adc1.Init.Direction            = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc            = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc               = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment  = DMA_PDATAALIGN_HALFWORD;
        hdma_adc1.Init.MemDataAlignment     = DMA_MDATAALIGN_HALFWORD;
        hdma_adc1.Init.Mode                 = DMA_CIRCULAR;
        hdma_adc1.Init.Priority             = DMA_PRIORITY_LOW;
        hdma_adc1.Init.FIFOMode             = DMA_FIFOMODE_DISABLE;

        if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
            _Error_Handler(__FILE__, __LINE__);
        }

        __HAL_LINKDMA(adcHandle,DMA_Handle,hdma_adc1);

        HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
    }

    else if(adcHandle->Instance==ADC2) {
        __HAL_RCC_ADC2_CLK_ENABLE();
    
        // ADC2 GPIO Configuration    
        // PC0     ------> ADC2_IN10
        // PC1     ------> ADC2_IN11
        // PC2     ------> ADC2_IN12
        // PC3     ------> ADC2_IN13
        // PA4     ------> ADC2_IN4
        // PA5     ------> ADC2_IN5
        // PA6     ------> ADC2_IN6
        // PC5     ------> ADC2_IN15 

        #ifndef USE_SINGLE_AXIS
            GPIO_InitStruct.Pin = M0_IB_Pin|M0_IC_Pin|M1_IC_Pin|M1_IB_Pin|M0_TEMP_Pin;
        #else
            GPIO_InitStruct.Pin = M0_IB_Pin|M0_IC_Pin|M0_TEMP_Pin;
        #endif
        
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        #ifndef USE_SINGLE_AXIS
            GPIO_InitStruct.Pin = M1_TEMP_Pin|AUX_TEMP_Pin|VBUS_S_Pin;
        #else
            GPIO_InitStruct.Pin = AUX_TEMP_Pin|VBUS_S_Pin;
        #endif

        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // ADC2 interrupt Init
        HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
    }

    else if(adcHandle->Instance==ADC3) {
        __HAL_RCC_ADC3_CLK_ENABLE();
    
        // ADC3 GPIO Configuration    
        // PC0     ------> ADC3_IN10
        // PC1     ------> ADC3_IN11
        // PC2     ------> ADC3_IN12
        // PC3     ------> ADC3_IN13 

        #ifndef USE_SINGLE_AXIS
            GPIO_InitStruct.Pin = M0_IB_Pin|M0_IC_Pin|M1_IC_Pin|M1_IB_Pin;
        #else
            GPIO_InitStruct.Pin = M0_IB_Pin|M0_IC_Pin;
        #endif

        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        // ADC3 interrupt Init
        HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
        HAL_NVIC_EnableIRQ(ADC_IRQn);
    }
}

// ---------------------------------------------------------------------------
// DSP_HAL_ADC_MspDeInit (already definied by Arduino.h)
//
//void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle) {
void DSP_HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle) {

    if(adcHandle->Instance==ADC1) {
        __HAL_RCC_ADC1_CLK_DISABLE();
    
        // ADC1 GPIO Configuration    
        // PC0     ------> ADC1_IN10
        // PC1     ------> ADC1_IN11
        // PC2     ------> ADC1_IN12
        // PC3     ------> ADC1_IN13
        // PA4     ------> ADC1_IN4
        // PA5     ------> ADC1_IN5
        // PA6     ------> ADC1_IN6
        // PC5     ------> ADC1_IN15 

        #ifndef USE_SINGLE_AXIS
            HAL_GPIO_DeInit(GPIOC, M0_IB_Pin|M0_IC_Pin|M1_IC_Pin|M1_IB_Pin 
                                    |M0_TEMP_Pin);

            HAL_GPIO_DeInit(GPIOA, M1_TEMP_Pin|AUX_TEMP_Pin|VBUS_S_Pin);
        #else
            HAL_GPIO_DeInit(GPIOC, M0_IB_Pin|M0_IC_Pin|M0_TEMP_Pin);

            HAL_GPIO_DeInit(GPIOA, AUX_TEMP_Pin|VBUS_S_Pin);
        #endif

        HAL_DMA_DeInit(adcHandle->DMA_Handle);
    }

    else if(adcHandle->Instance==ADC2) {
        __HAL_RCC_ADC2_CLK_DISABLE();

        // PC0     ------> ADC2_IN10
        // PC1     ------> ADC2_IN11
        // PC2     ------> ADC2_IN12
        // PC3     ------> ADC2_IN13
        // PA4     ------> ADC2_IN4
        // PA5     ------> ADC2_IN5
        // PA6     ------> ADC2_IN6
        // PC5     ------> ADC2_IN15 

        #ifndef USE_SINGLE_AXIS
            HAL_GPIO_DeInit(GPIOC, M0_IB_Pin|M0_IC_Pin|M1_IC_Pin|M1_IB_Pin 
                                    |M0_TEMP_Pin);

            HAL_GPIO_DeInit(GPIOA, M1_TEMP_Pin|AUX_TEMP_Pin|VBUS_S_Pin);
        #else
            HAL_GPIO_DeInit(GPIOC, M0_IB_Pin|M0_IC_Pin|M0_TEMP_Pin);

            HAL_GPIO_DeInit(GPIOA, AUX_TEMP_Pin|VBUS_S_Pin);
        #endif
    }

    else if(adcHandle->Instance==ADC3) {

        __HAL_RCC_ADC3_CLK_DISABLE();
    
        // ADC3 GPIO Configuration    
        // PC0     ------> ADC3_IN10
        // PC1     ------> ADC3_IN11
        // PC2     ------> ADC3_IN12
        // PC3     ------> ADC3_IN13 

        #ifndef USE_SINGLE_AXIS
            HAL_GPIO_DeInit(GPIOC, M0_IB_Pin|M0_IC_Pin|M1_IC_Pin|M1_IB_Pin);
        #else
            HAL_GPIO_DeInit(GPIOC, M0_IB_Pin|M0_IC_Pin);
        #endif
    }
} 


// ---------------------------------------------------------------------------
//
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
    //HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_4);
}


// ---------------------------------------------------------------------------
//
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
    //pwm_adc_state_tracker_ = 0;
    
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

// ----------------------------------------------------------------------------
// @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
// @ingroup low_level_fast
//

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

// ----------------------------------------------------------------------------
//
//ADC1_IRQHander


void (*adc_callback_)(bool, int);

extern "C" {

    void ADC_IRQHandler(void) {

        uint16_t now = TIM13->CNT;

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

            if(adc_callback_){
                adc_callback_(true, now);
            }

            // Clear interrupt flag & start next conversion
            __FAST__HAL_ADC_MODIFY_FLAG(ADC2, START_INJECTED_CONVERSION);        
        }

        // ADC2: Regular channel?
        else if(__FAST__HAL_ADC_GET_FLAG(ADC2, ADC_FLAG_EOC)) {

            if(adc_callback_){
                adc_callback_(false, now);
            }

            // Clear interrupt flag & start next conversion
            __FAST__HAL_ADC_MODIFY_FLAG(ADC2, START_REGULAR_CONVERSION);        
        }
    }
}


// ---------------------------------------------------------------------------
//
void start_adc_pwm() {

    // Enable ADC
    __HAL_ADC_ENABLE(&hadc1);
    __HAL_ADC_ENABLE(&hadc2);
    __HAL_ADC_ENABLE(&hadc3);
    
    // Wait for the ADC to enable
    // osDelay(2);

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
    //__HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
    //__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

    // Start brake resistor PWM in floating output configuration
    /*htim2.Instance->CCR3 = 0;
    htim2.Instance->CCR4 = TIM_APB1_PERIOD_CLOCKS + 1;
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);*/

    // Disarm motors and arm brake resistor
    //for (size_t i = 0; i < AXIS_COUNT; ++i) {
    //    safety_critical_disarm_motor_pwm(axes[i]->motor_);
    //}

    //safety_critical_arm_brake_resistor();
}


// ---------------------------------------------------------------------------
//
//
void DSP_set_adc_handler(void (*adc_callback)(bool, int)) {
    adc_callback_ = *adc_callback;
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

    // ADC GPIO
    DSP_HAL_ADC_MspInit(&hadc1);
    DSP_HAL_ADC_MspInit(&hadc2);
    DSP_HAL_ADC_MspInit(&hadc3);

    // Hack
    OC4_PWM_Override(&htim1);
    
    #ifndef USE_SINGLE_AXIS
        OC4_PWM_Override(&htim8);
    #endif

    // Start
    start_adc_pwm();
}