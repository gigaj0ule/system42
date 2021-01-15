#ifndef __USB_H__
    #define __USB_H__ 

	#include <stdint.h>
	#include <stdlib.h>

	#include <libopencm3/stm32/memorymap.h>
	
	#include "bootloader_config.h"

	// Exported API
	void usb_init();

#endif