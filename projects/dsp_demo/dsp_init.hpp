#ifndef __DSP_INIT_H__
    #define __DSP_INIT_H__

    #include <stm32f405xx.h>
    #include <stm32f4xx_hal_gpio.h>
    #include <stm32f4xx_hal_gpio_ex.h>
    #include <stm32f4xx_hal_adc.h>
    #include <stm32f4xx_hal_adc_ex.h>
    #include <stm32f4xx_hal_rcc.h>
    #include <stm32f4xx_hal_rcc_ex.h>
    #include <stm32f4xx_hal_tim.h>
    #include <stm32f4xx_hal_tim_ex.h>

    void DSP_TIM1_Init(void);
    void DSP_TIM3_Init(void);
    void DSP_TIM8_Init(void);
    void DSP_TIM4_Init(void);
    void DSP_TIM2_Init(void);
    void DSP_TIM5_Init(void);
    void DSP_TIM13_Init(void);
    void DSP_DMA_Init(void);
    void DSP_ADC1_Init(void);
    void DSP_ADC2_Init(void);
    void DSP_ADC3_Init(void);
    void DSP_HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle);
    void OC4_PWM_Override(TIM_HandleTypeDef* htim);
    void DSP_PWM_INIT(void);


    #ifndef M0_nCS_Pin

        #define M0_nCS_Pin                  GPIO_PIN_13
        #define M0_nCS_GPIO_Port            GPIOC

        #ifndef USE_SINGLE_AXIS
            #define M1_nCS_Pin              GPIO_PIN_14
            #define M1_nCS_GPIO_Port        GPIOC
            #define M1_ENC_Z_Pin            GPIO_PIN_15
            #define M1_ENC_Z_GPIO_Port      GPIOC
        #endif 

        #define M0_IB_Pin                   GPIO_PIN_0
        #define M0_IB_GPIO_Port             GPIOC
        #define M0_IC_Pin                   GPIO_PIN_1
        #define M0_IC_GPIO_Port             GPIOC

        #ifndef USE_SINGLE_AXIS
            #define M1_IC_Pin               GPIO_PIN_2
            #define M1_IC_GPIO_Port         GPIOC
            #define M1_IB_Pin               GPIO_PIN_3
            #define M1_IB_GPIO_Port         GPIOC
        #endif

        #define GPIO_1_Pin                  GPIO_PIN_0
        #define GPIO_1_GPIO_Port            GPIOA
        #define GPIO_2_Pin                  GPIO_PIN_1
        #define GPIO_2_GPIO_Port            GPIOA
        #define GPIO_3_Pin                  GPIO_PIN_2
        #define GPIO_3_GPIO_Port            GPIOA
        #define GPIO_4_Pin                  GPIO_PIN_3
        #define GPIO_4_GPIO_Port            GPIOA

        #ifndef USE_SINGLE_AXIS
            #define M1_TEMP_Pin             GPIO_PIN_4
            #define M1_TEMP_GPIO_Port       GPIOA
        #endif 

        #define AUX_TEMP_Pin                GPIO_PIN_5
        #define AUX_TEMP_GPIO_Port          GPIOA
        #define VBUS_S_Pin                  GPIO_PIN_6
        #define VBUS_S_GPIO_Port            GPIOA

        #ifndef USE_SINGLE_AXIS
            #define M1_AL_Pin               GPIO_PIN_7
            #define M1_AL_GPIO_Port         GPIOA
        #endif 

        #define GPIO_5_Pin                  GPIO_PIN_4
        #define GPIO_5_GPIO_Port            GPIOC
        #define M0_TEMP_Pin                 GPIO_PIN_5
        #define M0_TEMP_GPIO_Port           GPIOC

        #ifndef USE_SINGLE_AXIS
            #define M1_BL_Pin               GPIO_PIN_0
            #define M1_BL_GPIO_Port         GPIOB
            #define M1_CL_Pin               GPIO_PIN_1
            #define M1_CL_GPIO_Port         GPIOB
        #endif

        #define GPIO_6_Pin                  GPIO_PIN_2
        #define GPIO_6_GPIO_Port            GPIOB
        #define AUX_L_Pin                   GPIO_PIN_10
        #define AUX_L_GPIO_Port             GPIOB
        #define AUX_H_Pin                   GPIO_PIN_11
        #define AUX_H_GPIO_Port             GPIOB
        #define EN_GATE_Pin                 GPIO_PIN_12
        #define EN_GATE_GPIO_Port           GPIOB
        #define M0_AL_Pin                   GPIO_PIN_13
        #define M0_AL_GPIO_Port             GPIOB
        #define M0_BL_Pin                   GPIO_PIN_14
        #define M0_BL_GPIO_Port             GPIOB
        #define M0_CL_Pin                   GPIO_PIN_15
        #define M0_CL_GPIO_Port             GPIOB

        #ifndef USE_SINGLE_AXIS
            #define M1_AH_Pin               GPIO_PIN_6
            #define M1_AH_GPIO_Port         GPIOC
            #define M1_BH_Pin               GPIO_PIN_7
            #define M1_BH_GPIO_Port         GPIOC
            #define M1_CH_Pin               GPIO_PIN_8
            #define M1_CH_GPIO_Port         GPIOC
        #endif 

        #define M0_ENC_Z_Pin                GPIO_PIN_9
        #define M0_ENC_Z_GPIO_Port          GPIOC
        #define M0_AH_Pin                   GPIO_PIN_8
        #define M0_AH_GPIO_Port             GPIOA
        #define M0_BH_Pin                   GPIO_PIN_9
        #define M0_BH_GPIO_Port             GPIOA
        #define M0_CH_Pin                   GPIO_PIN_10
        #define M0_CH_GPIO_Port             GPIOA
        #define GPIO_7_Pin                  GPIO_PIN_15
        #define GPIO_7_GPIO_Port            GPIOA

        #ifdef USE_MOTO_PINS
            #define nFAULT_Pin              GPIO_PIN_1
            #define nFAULT_GPIO_Port        GPIOB
        #else
            #define nFAULT_Pin              GPIO_PIN_2
            #define nFAULT_GPIO_Port        GPIOD
        #endif

        #define GPIO_8_Pin                  GPIO_PIN_3
        #define GPIO_8_GPIO_Port            GPIOB
        #define M0_ENC_A_Pin                GPIO_PIN_4
        #define M0_ENC_A_GPIO_Port          GPIOB
        #define M0_ENC_B_Pin                GPIO_PIN_5
        #define M0_ENC_B_GPIO_Port          GPIOB

        #ifndef USE_SINGLE_AXIS
            #define M1_ENC_A_Pin            GPIO_PIN_6
            #define M1_ENC_A_GPIO_Port      GPIOB
            #define M1_ENC_B_Pin            GPIO_PIN_7
            #define M1_ENC_B_GPIO_Port      GPIOB
        #endif

    #endif


#endif