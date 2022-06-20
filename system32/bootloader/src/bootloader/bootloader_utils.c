#include <stdint.h>

#include <libopencm3/stm32/tools.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/desig.h>

#include "bootloader_utils.h"
#include "bootloader_usb.h"

// =========================================================================
// Implement this here to save space, quite minimalistic :D
void *memcpy(void * dst, const void * src, size_t count) {
	uint8_t * dstb = (uint8_t*)dst;
	uint8_t * srcb = (uint8_t*)src;
	while (count--) {
		*dstb++ = *srcb++;
	}
	return dst;
}

// =========================================================================
// Returns length of string
size_t strlen(const char *s) {
	size_t ret = 0;
	while (*s++) {
		ret++;
	}
	return ret;
}

// =========================================================================
// Returns flash page size
#ifdef STM32F4
	// Sector size map for STM32F4
	uint32_t f40x_sector_sizes [12] = 
	{16384, 	16384, 		16384, 		16384, 		65536,
	131072, 	131072, 	131072, 	131072, 	131072, 
	131072, 	131072};
#endif

uint32_t getFlashSectorSize(uint8_t sector) {

	#ifdef STM32F0  // Only valid for STM32F072
	    //uint32_t flash_size = 1024 * desig_get_flash_size();
		//if (flash_size > (128 * 1024)) {
			// STM Datasheet says 2kB for big devices
			return 0x800;
		//}
		//else {
			// 1kB for small ones
		//	return 0x400;
		//}
	#elif defined(STM32F1)
	    uint32_t flash_size = 1024 * desig_get_flash_size();

		if (flash_size > (128 * 1024) + 1) {
			// STM Datasheet says 2k/B for big devices
			return 0x800;
		}
		else {
			// 1kB for small ones
			return 0x400;
		}
	#elif defined(STM32F4) 
		// NOTE: The F4 has variable flash page size so this 
		// code only returns correct value for the last sectors
		// of flash!
		return f40x_sector_sizes[sector];
	#endif
}

// =========================================================================
// Convert HEX to string
void atoh(char *ascii_ptr, char *hex_ptr,int len) {
    int i;

    for(i = 0; i < (len / 2); i++) {
        *(hex_ptr+i)   = (*(ascii_ptr+(2*i)) <= '9') ? ((*(ascii_ptr+(2*i)) - '0') * 16 ) :  (((*(ascii_ptr+(2*i)) - 'A') + 10) << 4);
        *(hex_ptr+i)  |= (*(ascii_ptr+(2*i)+1) <= '9') ? (*(ascii_ptr+(2*i)+1) - '0') :  (*(ascii_ptr+(2*i)+1) - 'A' + 10);
    }
}

// =========================================================================
// Resets the USB bus by forcibly shorting D+ 
void usb_hard_reset() {
	
	// Forcibly pull d+ low. Need at least 2.5us to trigger USB disconnect
	#if defined(STM32F1)
		// Disable USB peripheral as it overrides GPIO settings

		#warning fixme
		//*USB_CNTR_REG = USB_CNTR_PWDN;

		// Pull down d+
		gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
		gpio_clear(GPIOA, 12);
	#elif defined(STM32F4)
		gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
		gpio_clear(GPIOA, 12);
	#endif
	
	// Delay a bit to wait for reset
	for (unsigned int i = 0; i < 100000; i++) {
		__asm__("nop");
	}
}

// =========================================================================
// Jump to an application by setting the program counter and stack pointer 
// to a new interrupt vector table at app_entry
void jump_to_application(uint32_t app_entry) {

    // Dedicated function with no call to any function (appart the last call)
    // This way, there is no manipulation of the stack here, ensuring that GGC
    // didn't insert any pop from the SP after having set the MSP.
	
    typedef void (*funcPtr)(void);
    
    // Where is application entry point()? 
	// It should be at the stack pointer entry in the application interrupt vector table.
    uint32_t jumpAddr = *(volatile uint32_t *)(app_entry + 0x04); 

	// Create a function pointer to the entry point
    funcPtr usrMain = (funcPtr) jumpAddr;

	// Tell the MCU where the new interrupt vector table is
    SET_REG(&SCB_VTOR, (volatile uint32_t) (app_entry));

	// Set the stack pointer to app_entry
    __asm__ volatile ("msr msp, %0"::"g"(*(volatile uint32_t *)app_entry));

    // go!
    usrMain();
}

// =========================================================================
// Hard-reset the mirco controller by writing to the ARM
// system control register
void system_hard_reset() {

	// Write an invalid state to some registers
	volatile uint32_t *_scb_aircr = (uint32_t*)0xE000ED0CU;
	*_scb_aircr = 0x05FA0000 | 0x4;

	while(1);
	
	__builtin_unreachable();
}
