#ifndef __DSP_FN_HPP__
    #define __DSP_FN_HPP__

    #include <stm32f405xx.h>
    #include <stm32f4xx_hal.h>      // Sets up the correct chip specifc defines required by arm_math
    #include <stm32f4xx_hal_tim.h>  // Sets up the correct chip specifc defines required by arm_math

    #ifndef TIM_1_8_CLOCK_HZ

        #define USE_SINGLE_AXIS

        #define TIM_1_8_CLOCK_HZ            168000000
        #define TIM_1_8_PERIOD_CLOCKS       3500
        #define TIM_1_8_DEADTIME_CLOCKS     200
        #define TIM_APB1_CLOCK_HZ           84000000
        #define TIM_APB1_PERIOD_CLOCKS      4096
        #define TIM_APB1_DEADTIME_CLOCKS    200
        #define TIM_1_8_RCR                 0

        #define CURRENT_MEAS_PERIOD (float)((float)2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1) / (float)TIM_1_8_CLOCK_HZ ) * (float)(3 - (TIM_1_8_RCR))
        #define CURRENT_MEAS_HZ  (float)((float)(TIM_1_8_CLOCK_HZ) / (float)(2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1))) / (float)(3 - (TIM_1_8_RCR))

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
    #endif

    #define TIMER_ONE_ARRAY_INDEX 0
    #define TIMER_EIGHT_ARRAY_INDEX 0

    void DSP_setup(void);

    void DSP_set_adc_sample_complete_callback(void (*adc_sample_complete_callback)(bool));
    void DSP_set_timer_one_control_loop_callback(void (*timer_one_control_loop_callback)());
    void DSP_set_timer_eight_control_loop_callback(void (*timer_eight_control_loop_callback)());

    extern "C" {
        void ADC_IRQHandler(void);
        void TIM1_UP_TIM10_IRQHandler(void);
        void TIM8_UP_TIM13_IRQHandler(void);
    }

#endif