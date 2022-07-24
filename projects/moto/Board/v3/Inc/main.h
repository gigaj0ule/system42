/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"

#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 1 \
||  HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 2
#include "prev_board_ver/main_V3_2.h"
#elif HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 3 \
||  HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR == 4
#include "prev_board_ver/main_V3_4.h"
#else
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define TIM_1_8_CLOCK_HZ 168000000
#define TIM_1_8_PERIOD_CLOCKS 3500
#define TIM_1_8_DEADTIME_CLOCKS 200
#define TIM_APB1_CLOCK_HZ 84000000
#define TIM_APB1_PERIOD_CLOCKS 4096
#define TIM_APB1_DEADTIME_CLOCKS 200
#define TIM_1_8_RCR 0

#define M0_nCS_Pin GPIO_PIN_13
#define M0_nCS_GPIO_Port GPIOC

#ifndef USE_SINGLE_AXIS
  #define M1_nCS_Pin GPIO_PIN_14
  #define M1_nCS_GPIO_Port GPIOC
  #define M1_ENC_Z_Pin GPIO_PIN_15
  #define M1_ENC_Z_GPIO_Port GPIOC
#endif 

#define M0_IB_Pin GPIO_PIN_0
#define M0_IB_GPIO_Port GPIOC
#define M0_IC_Pin GPIO_PIN_1
#define M0_IC_GPIO_Port GPIOC

#ifndef USE_SINGLE_AXIS
  #define M1_IC_Pin GPIO_PIN_2
  #define M1_IC_GPIO_Port GPIOC
  #define M1_IB_Pin GPIO_PIN_3
  #define M1_IB_GPIO_Port GPIOC
#endif

#define GPIO_1_Pin GPIO_PIN_0
#define GPIO_1_GPIO_Port GPIOA
#define GPIO_2_Pin GPIO_PIN_1
#define GPIO_2_GPIO_Port GPIOA
#define GPIO_3_Pin GPIO_PIN_2
#define GPIO_3_GPIO_Port GPIOA
#define GPIO_4_Pin GPIO_PIN_3
#define GPIO_4_GPIO_Port GPIOA

#ifndef USE_SINGLE_AXIS
  #define M1_TEMP_Pin GPIO_PIN_4
  #define M1_TEMP_GPIO_Port GPIOA
#endif 

#define AUX_TEMP_Pin GPIO_PIN_5
#define AUX_TEMP_GPIO_Port GPIOA
#define VBUS_S_Pin GPIO_PIN_6
#define VBUS_S_GPIO_Port GPIOA

#ifndef USE_SINGLE_AXIS
  #define M1_AL_Pin GPIO_PIN_7
  #define M1_AL_GPIO_Port GPIOA
#endif 

#define GPIO_5_Pin GPIO_PIN_4
#define GPIO_5_GPIO_Port GPIOC
#define M0_TEMP_Pin GPIO_PIN_5
#define M0_TEMP_GPIO_Port GPIOC

#ifndef USE_SINGLE_AXIS
  #define M1_BL_Pin GPIO_PIN_0
  #define M1_BL_GPIO_Port GPIOB
  #define M1_CL_Pin GPIO_PIN_1
  #define M1_CL_GPIO_Port GPIOB
#endif

#define GPIO_6_Pin GPIO_PIN_2
#define GPIO_6_GPIO_Port GPIOB
#define AUX_L_Pin GPIO_PIN_10
#define AUX_L_GPIO_Port GPIOB
#define AUX_H_Pin GPIO_PIN_11
#define AUX_H_GPIO_Port GPIOB
#define EN_GATE_Pin GPIO_PIN_12
#define EN_GATE_GPIO_Port GPIOB
#define M0_AL_Pin GPIO_PIN_13
#define M0_AL_GPIO_Port GPIOB
#define M0_BL_Pin GPIO_PIN_14
#define M0_BL_GPIO_Port GPIOB
#define M0_CL_Pin GPIO_PIN_15
#define M0_CL_GPIO_Port GPIOB

#ifndef USE_SINGLE_AXIS
  #define M1_AH_Pin GPIO_PIN_6
  #define M1_AH_GPIO_Port GPIOC
  #define M1_BH_Pin GPIO_PIN_7
  #define M1_BH_GPIO_Port GPIOC
  #define M1_CH_Pin GPIO_PIN_8
  #define M1_CH_GPIO_Port GPIOC
#endif 

#define M0_ENC_Z_Pin GPIO_PIN_9
#define M0_ENC_Z_GPIO_Port GPIOC
#define M0_AH_Pin GPIO_PIN_8
#define M0_AH_GPIO_Port GPIOA
#define M0_BH_Pin GPIO_PIN_9
#define M0_BH_GPIO_Port GPIOA
#define M0_CH_Pin GPIO_PIN_10
#define M0_CH_GPIO_Port GPIOA
#define GPIO_7_Pin GPIO_PIN_15
#define GPIO_7_GPIO_Port GPIOA

#ifdef USE_MOTO_PINS
  #define nFAULT_Pin GPIO_PIN_1
  #define nFAULT_GPIO_Port GPIOB
#else
  #define nFAULT_Pin GPIO_PIN_2
  #define nFAULT_GPIO_Port GPIOD
#endif

#define GPIO_8_Pin GPIO_PIN_3
#define GPIO_8_GPIO_Port GPIOB
#define M0_ENC_A_Pin GPIO_PIN_4
#define M0_ENC_A_GPIO_Port GPIOB
#define M0_ENC_B_Pin GPIO_PIN_5
#define M0_ENC_B_GPIO_Port GPIOB

#ifndef USE_SINGLE_AXIS
  #define M1_ENC_A_Pin GPIO_PIN_6
  #define M1_ENC_A_GPIO_Port GPIOB
  #define M1_ENC_B_Pin GPIO_PIN_7
  #define M1_ENC_B_GPIO_Port GPIOB
#endif

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#endif

//TODO: make this come automatically out of CubeMX somehow
#define TIM_TIME_BASE TIM14

#define CURRENT_MEAS_PERIOD (float)((float)2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1) / (float)TIM_1_8_CLOCK_HZ ) * (float)(3 - (TIM_1_8_RCR))
#define CURRENT_MEAS_HZ  (float)((float)(TIM_1_8_CLOCK_HZ) / (float)(2*TIM_1_8_PERIOD_CLOCKS*(TIM_1_8_RCR+1))) / (float)(3 - (TIM_1_8_RCR))

#ifdef USE_HIGH_VOLTAGE
#define VBUS_S_DIVIDER_RATIO 41.02f
#define VBUS_OVERVOLTAGE_LEVEL 90.0f
#elif HW_VERSION_VOLTAGE == 48
#define VBUS_S_DIVIDER_RATIO 19.0f
#define VBUS_OVERVOLTAGE_LEVEL 52.0f
#elif HW_VERSION_VOLTAGE == 24
#define VBUS_S_DIVIDER_RATIO 11.0f
#define VBUS_OVERVOLTAGE_LEVEL 26.0f
#else
#error "unknown board voltage"
#endif

#define ADC_FULL_SCALE (1 << 12)
#define ADC_HALF_SCALE (1 << 11)
#define ADC_REF_VOLTAGE 3.3f
#define ADC_VOLTS_PER_COUNT ADC_REF_VOLTAGE / (float)ADC_FULL_SCALE
#define ADC_VBUS_VOLTAGE_SCALE ADC_REF_VOLTAGE * VBUS_S_DIVIDER_RATIO / (float)ADC_FULL_SCALE

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
