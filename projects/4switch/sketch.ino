// Headers
#include "sketch.hpp"
#include "sketch_only_statics.hpp"
#include "Wire.h"

#include "protocol.hpp"

#if defined(USBD_USE_HID_COMPOSITE)
	#include "Keyboard.h"
	#include "Mouse.h"
#endif

const uint8_t g_LP_cie_8bit[513] = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 2, 2,
	2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
	2, 2, 2, 2, 2, 2, 3, 3, 3, 3,
	3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
	3, 4, 4, 4, 4, 4, 4, 4, 4, 4,
	4, 4, 4, 5, 5, 5, 5, 5, 5, 5,
	5, 5, 5, 5, 6, 6, 6, 6, 6, 6,
	6, 6, 6, 7, 7, 7, 7, 7, 7, 7,
	7, 7, 8, 8, 8, 8, 8, 8, 8, 8,
	9, 9, 9, 9, 9, 9, 9, 10, 10, 10,
	10, 10, 10, 10, 11, 11, 11, 11, 11, 11,
	12, 12, 12, 12, 12, 12, 13, 13, 13, 13,
	13, 13, 14, 14, 14, 14, 14, 15, 15, 15,
	15, 15, 16, 16, 16, 16, 16, 17, 17, 17,
	17, 17, 18, 18, 18, 18, 19, 19, 19, 19,
	19, 20, 20, 20, 20, 21, 21, 21, 21, 22,
	22, 22, 22, 23, 23, 23, 23, 24, 24, 24,
	24, 25, 25, 25, 26, 26, 26, 26, 27, 27,
	27, 28, 28, 28, 28, 29, 29, 29, 30, 30,
	30, 31, 31, 31, 32, 32, 32, 33, 33, 33,
	33, 34, 34, 35, 35, 35, 36, 36, 36, 37,
	37, 37, 38, 38, 38, 39, 39, 39, 40, 40,
	41, 41, 41, 42, 42, 43, 43, 43, 44, 44,
	45, 45, 45, 46, 46, 47, 47, 47, 48, 48,
	49, 49, 50, 50, 50, 51, 51, 52, 52, 53,
	53, 54, 54, 54, 55, 55, 56, 56, 57, 57,
	58, 58, 59, 59, 60, 60, 61, 61, 62, 62,
	63, 63, 64, 64, 65, 65, 66, 66, 67, 67,
	68, 68, 69, 69, 70, 71, 71, 72, 72, 73,
	73, 74, 74, 75, 76, 76, 77, 77, 78, 78,
	79, 80, 80, 81, 81, 82, 83, 83, 84, 84,
	85, 86, 86, 87, 88, 88, 89, 89, 90, 91,
	91, 92, 93, 93, 94, 95, 95, 96, 97, 97,
	98, 99, 99, 100, 101, 102, 102, 103, 104, 104,
	105, 106, 106, 107, 108, 109, 109, 110, 111, 112,
	112, 113, 114, 115, 115, 116, 117, 118, 118, 119,
	120, 121, 122, 122, 123, 124, 125, 126, 126, 127,
	128, 129, 130, 130, 131, 132, 133, 134, 135, 135,
	136, 137, 138, 139, 140, 141, 141, 142, 143, 144,
	145, 146, 147, 148, 148, 149, 150, 151, 152, 153,
	154, 155, 156, 157, 158, 159, 159, 160, 161, 162,
	163, 164, 165, 166, 167, 168, 169, 170, 171, 172,
	173, 174, 175, 176, 177, 178, 179, 180, 181, 182,
	183, 184, 185, 186, 187, 188, 189, 191, 192, 193,
	194, 195, 196, 197, 198, 199, 200, 201, 202, 204,
	205, 206, 207, 208, 209, 210, 211, 213, 214, 215,
	216, 217, 218, 219, 221, 222, 223, 224, 225, 227,
	228, 229, 230, 231, 233, 234, 235, 236, 237, 239,
	240, 241, 242, 244, 245, 246, 247, 249, 250, 251,
	252, 254, 255
};

//#define PIN_I2C_SCL1 PB6
//#define PIN_I2C_SDA1 PB7
//#define ADDRESS_SM72442 0x01


#include <stm32f1xx_hal_gpio.h>
#include <stm32f1xx_hal_adc.h>
#include <stm32f1xx_hal_rcc.h>
#include <stm32f1xx_hal_tim.h>
#include <interrupt.h>

//Output frequency (Hz)
#define OUTPUT_FREQ		150000

//Duty cycle (%)
#define DUTY_CYCLE		75

TIM_HandleTypeDef 				htim1;
TIM_ClockConfigTypeDef 			sClockSourceConfig;
TIM_MasterConfigTypeDef 		sMasterConfig;
TIM_OC_InitTypeDef 				sConfigOC;
TIM_BreakDeadTimeConfigTypeDef 	sBreakDeadTimeConfig;

ADC_HandleTypeDef 				hadc1;
ADC_InjectionConfTypeDef 		sConfigInjected;

#define TIM1_PERIOD (SystemCoreClock / OUTPUT_FREQ)

void set_pwm(TIM_HandleTypeDef * htim, int a, int b, int c) 
{
	htim->Instance->CCR1 = map(a, 0, 255, 0, TIM1_PERIOD);
	htim->Instance->CCR2 = map(b, 0, 255, 0, TIM1_PERIOD);
	htim->Instance->CCR3 = map(c, 0, 255, 0, TIM1_PERIOD);
	htim->Instance->CCR4 = 1;
}

// @breif Starts the desired PWM timer
// @ingroup low_level
void start_pwm(TIM_HandleTypeDef* htim) 
{
	// Preload the timer registers to 50%
	// except for channel 4 which we don't use for PWM
	set_pwm(htim, 10, 128, 128);

	// Start up all the timers in PWM mode using the HAL
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_2);
	//HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
	//HAL_TIMEx_PWMN_Start(htim, TIM_CHANNEL_3);

	// Ecept for channel 4... wait why do we bother with this?
	// HAL_TIM_PWM_Start_IT(htim, TIM_CHANNEL_4);
}


static void worker_thread(void* arg) 
{
	int i = 0;
	int j = 1;

	while(true) {
		// SM72242 Loop Task
		os_delay(1);

		i += j;

		if(i > 511) j = -1;
		if(i < 1) j = 1;

		set_pwm(&htim1, g_LP_cie_8bit[i], g_LP_cie_8bit[512-i], 0);
	}
}

void setup() {

	#if defined(USBD_USE_HID_COMPOSITE)
		Mouse.begin();
		Keyboard.begin();
	#endif
		
	// Init communication
	early_setup();

	// TIM1 
	__HAL_RCC_TIM1_CLK_ENABLE();

	htim1.Instance 					= TIM1;
	htim1.Init.Prescaler 			= 0;
	htim1.Init.CounterMode 			= TIM_COUNTERMODE_CENTERALIGNED3;
	htim1.Init.Period 				= SystemCoreClock/OUTPUT_FREQ;
	htim1.Init.ClockDivision 		= TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter 	= 3;

	if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);
	
	sClockSourceConfig.ClockSource 	= TIM_CLOCKSOURCE_INTERNAL;
	
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);
	
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	sMasterConfig.MasterOutputTrigger 	= TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode 		= TIM_MASTERSLAVEMODE_DISABLE;

	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	sConfigOC.OCMode 		= TIM_OCMODE_PWM2;
	sConfigOC.Pulse 		= 0;
	sConfigOC.OCPolarity 	= TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity 	= TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode 	= TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState 	= TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState 	= TIM_OCNIDLESTATE_RESET;

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	sConfigOC.OCMode = TIM_OCMODE_TIMING;

	if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	sBreakDeadTimeConfig.OffStateRunMode 	= TIM_OSSR_ENABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode 	= TIM_OSSI_ENABLE;
	sBreakDeadTimeConfig.LockLevel 			= TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime 			= 3;
	sBreakDeadTimeConfig.BreakState 		= TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity 		= TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.AutomaticOutput 	= TIM_AUTOMATICOUTPUT_DISABLE;

	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	// GPIO
	__HAL_RCC_GPIOA_CLK_ENABLE();
  	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef GPIO_InitStruct;

	/**TIM1 GPIO Configuration	
	PB13	 ------> TIM1_CH1N
	PB14	 ------> TIM1_CH2N
	PB15	 ------> TIM1_CH3N
	PA8	 ------> TIM1_CH1
	PA9	 ------> TIM1_CH2
	PA10	 ------> TIM1_CH3 
	*/
	GPIO_InitStruct.Pin 		= GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_LOW;
	//GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin 		= GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
	GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull 		= GPIO_NOPULL;
	GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_LOW;
	//GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Begin PWM
	start_pwm(&htim1);

	/* Interrupt TIM1 */
	// configure Update interrupt
	//HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 5, 0);
	//HAL_NVIC_EnableIRQ(TIM1_IRQn);

	// ADC Setup
	hadc1.Instance 						= ADC1;
	hadc1.Init.ScanConvMode 			= DISABLE;
	hadc1.Init.ContinuousConvMode 		= DISABLE;
	hadc1.Init.DiscontinuousConvMode 	= DISABLE;
	hadc1.Init.ExternalTrigConv 		= ADC_SOFTWARE_START;
	hadc1.Init.DataAlign 				= ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion 			= 1;

	if (HAL_ADC_Init(&hadc1) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	sConfigInjected.InjectedChannel 				= ADC_CHANNEL_6;
	sConfigInjected.InjectedRank 					= 1;
	sConfigInjected.InjectedNbrOfConversion 		= 1;
	sConfigInjected.InjectedSamplingTime 			= ADC_SAMPLETIME_3CYCLES;
	sConfigInjected.ExternalTrigInjecConv 			= ADC_EXTERNALTRIGINJECCONV_T1_TRGO;
	sConfigInjected.AutoInjectedConv 				= DISABLE;
	sConfigInjected.InjectedDiscontinuousConvMode 	= DISABLE;
	sConfigInjected.InjectedOffset 					= 0;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
		_Error_Handler(__FILE__, __LINE__);

	HAL_ADC_Start(&hadc1);

	// IRQ?
	HAL_NVIC_SetPriority(ADC_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);


	// Launch program!
	create_threads(worker_thread);
};


void loop(){	
	__asm__ volatile("nop");
	//os_delay(1);
	
	#if defined(USBD_USE_CDC)
	//SerialUSB.print("Hi");
	#endif
};