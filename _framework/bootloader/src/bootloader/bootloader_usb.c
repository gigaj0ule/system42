#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/usb/usbd.h>

#if defined(STM32F0) || defined(STM32F1)
	#include <libopencm3/stm32/st_usbfs.h>
#elif defined(STM32F4)
	
#endif

#include <libopencm3/usb/usbstd.h>

#include "bootloader_config.h"
#include "bootloader_utils.h"
#include "bootloader_usb.h"
#include "bootloader_dfuse.h"
#include "bootloader_usb_descriptor.h"
#include "bootloader_tests.h"

// =========================================================================
// Defined in main
extern uint8_t usbd_control_buffer[1024];

// =========================================================================
// Defined in descriptor.c
extern const struct usb_device_descriptor dev_desc;
extern const char * const _usb_strings[4];
extern const uint8_t _num_strings;

extern const struct usb_config_descriptor usb_configuration_descriptor;
extern struct usb_dfu_descriptor dfu_function;

extern const char iSerial[32];
extern struct iFlashMemoryPartitions_t iFlashMemoryPartitions;

extern uint32_t writable_flash_end_address_;
extern uint16_t wTransferSize_;
extern uint32_t flash_end_address_;

usbd_device* usbd_dev = NULL;

// =========================================================================
void usb_init() {
	
	// Set wTransferSize_ to be 1kB
	wTransferSize_ = 1024;

	dfu_function.wTransferSize = wTransferSize_;
	
	// Now get flash total size
	uint32_t flash_size = 1024 * desig_get_flash_size();
	uint32_t writable_flash_size = flash_size;

	#ifdef PROTECT_LAST_TWO_FLASH_PAGES
		// Get flash sector size
		uint32_t flash_sector_size = getFlashSectorSize(10);
		// Omit them from writable flash
		writable_flash_size -= (2 * flash_sector_size);
	#endif

	// Save flash end address
	flash_end_address_ = flash_size + FLASH_BASE_ADDR;
	writable_flash_end_address_ = writable_flash_size + FLASH_BASE_ADDR;

	// We hard code our payload block size to 1024. Even though ST's spec recommends
	// to use the sector size, I found that in practice it's easier to just assume 1kB.
	#ifdef ENABLE_AES_DECRYPTION
		uint32_t payload_sector_size = writable_flash_size / 1024;
	#else
		uint32_t payload_sector_size = (writable_flash_size + (FLASH_BOOTLDR_SIZE_KB_INT * 1024)) / 1024;	
	#endif

	// Populate flash partition string descriptor
	iFlashMemoryPartitions.payload_size_kb[0] = ((payload_sector_size / 10000) % 10) + '0';
	iFlashMemoryPartitions.payload_size_kb[1] = ((payload_sector_size / 1000) % 10) + '0';
	iFlashMemoryPartitions.payload_size_kb[2] = ((payload_sector_size / 100) % 10) + '0';
	iFlashMemoryPartitions.payload_size_kb[3] = ((payload_sector_size / 10)  % 10) + '0';
	iFlashMemoryPartitions.payload_size_kb[4] = ((payload_sector_size / 1)   % 10) + '0';
	
	// Set the device serial nummber from the chip ROM
	print_device_serial_number((char *) iSerial);

	// Init USB Clocks and AF pins (f4)
	#ifdef STM32F4
		rcc_periph_clock_enable(RCC_OTGFS);
		gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
		gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);

		// Disable Vbus Sense
		#include <libopencm3/usb/dwc/otg_common.h>
		#include <libopencm3/usb/dwc/otg_fs.h>
		OTG_FS_GCCFG |= OTG_GCCFG_NOVBUSSENS | OTG_GCCFG_PWRDWN; 
	#endif

	// Init USB Driver
	#if defined(STM32F0) || defined(STM32F1)
		usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_desc, &usb_configuration_descriptor, 
			_usb_strings, _num_strings,
			usbd_control_buffer, sizeof(usbd_control_buffer));
	#elif defined(STM32F4)
		usbd_dev = usbd_init(&otgfs_usb_driver, &dev_desc, &usb_configuration_descriptor,
			_usb_strings, _num_strings,
			usbd_control_buffer, sizeof(usbd_control_buffer));
	#else
		#error No support for other chips yet
	#endif

	// Register USB DFU control callback
	const uint8_t dfu_type = USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE;
	const uint8_t dfu_mask = USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT;
	usbd_register_control_callback(usbd_dev, dfu_type, dfu_mask, usbdfu_control_request);
};