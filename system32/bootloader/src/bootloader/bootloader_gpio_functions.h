#ifndef __GPIO_H__
	#define __GPIO_H__

	#include <stdint.h>

	#include <libopencm3/stm32/memorymap.h>
	#include <libopencm3/stm32/gpio.h>
	#include <libopencm3/stm32/rcc.h>
    #include "bootloader_config.h"

	// =========================================================================
	static void set_up_led_and_button(void) {
		
		#if defined(STM32F0)

		#elif defined(STM32F1)
		
			gpio_set_mode(LED_BANK, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);

			#if defined(BUTTON_BANK) && defined (BUTTON_PIN)
				gpio_set_mode(BUTTON_BANK, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, BUTTON_PIN);
				gpio_clear(BUTTON_BANK, BUTTON_PIN);
			#endif
		
		#elif defined(STM32F4)

			gpio_mode_setup(LED_BANK, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LED_PIN);

			#if defined(BUTTON_BANK) && defined (BUTTON_PIN)
				gpio_mode_setup(BUTTON_BANK, GPIO_MODE_INPUT, GPIO_PUPD_PULLDOWN, BUTTON_PIN);
				gpio_clear(BUTTON_BANK, BUTTON_PIN);
			#endif

		#endif
	}

#endif