#ifndef __FLASH_H__
	#define __FLASH_H__

	#include <libopencm3/stm32/flash.h>

	#include "bootloader_config.h"

	// =========================================================================
	/*static int _flash_page_is_erased(uint32_t addr) {
		volatile uint32_t *_ptr32 = (uint32_t*)addr;
		// Todo: support variable size flash pages
		for (unsigned i = 0; i < 1024/sizeof(uint32_t); i++) {
			if (_ptr32[i] != 0xffffffffU) {
				return 0;
			}
		}
		return 1;
	}*/

	// =========================================================================
	static inline void _flash_program_buffer(uint32_t address, uint8_t * data, uint32_t len) {
		flash_wait_for_last_operation();
		
		#ifdef STM32F4
			//flash_set_program_size(FLASH_CR_PROGRAM_X16);
			//flash_wait_for_last_operation();
		#endif

		// Stm32f1 prefers 16 bit flash word writes,
		// Looks like the f4 has this option also
		for (uint32_t i = 0; i < len; i+=2) {
			volatile uint32_t address_index = address + i;
			volatile uint16_t data_hword = ((data[i+1] << 8) | data[i]);
			flash_program_half_word(address_index, data_hword);
		}
	}

#endif
