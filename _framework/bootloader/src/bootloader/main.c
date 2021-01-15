#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "bootloader_tests.h"
#include "bootloader_utils.h"
#include "bootloader_flash_functions.h"
#include "bootloader_gpio_functions.h"
#include "bootloader_dfuse.h"
#include "bootloader_usb_descriptor.h"
#include "bootloader_usb.h"

#include <libopencm3/usb/usbd.h>

extern enum dfu_state dfu_state_;
extern uint32_t application_start_address_;
extern usbd_device* usbd_dev;

// Boot the application if it seems valid and we haven't been
// asked to reboot into DFU mode. This should make the CPU to 
// boot into DFU if the user app has been erased.
int main(void) {

	// Enable clocks
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOD);

	// Enable basic I/O
	set_up_led_and_button();

	// Test to see if this is was a dumb clone
	check_set_device_unique();

	#if defined(ENABLE_READOUT_PROTECTION_LEVEL_1)
		// Enable read-out protection
		check_set_rdp();
	#endif

	// Do some tests to see if we should enter DFU mode
	volatile bool button_state = test_button_state();
	volatile bool app_is_missing = test_if_user_app_is_missing(application_start_address_);
	volatile bool boot_token_in_ram = dfu_boot_token();

	bool enter_dfu_mode = boot_token_in_ram || app_is_missing || button_state;

	// Should we enter the application?   
	if (!enter_dfu_mode) {
		jump_to_application(application_start_address_);
	}

	// Set up high speed clocks
	#ifdef STM32F1
		rcc_clock_setup_in_hse_8mhz_out_72mhz();
	#elif defined(STM32F4)
		rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
	#endif

	// Hard reset USB connection
	usb_hard_reset();

	// Init USB
	usb_init();

	uint32_t i = 0;
	
	// Wait for activity...
	while (true) {

		// Blink LED to signify we are in bootloader mode
		i++;
		if(i % 50000UL == 0 || i % 55500UL == 0) {
			gpio_toggle(LED_BANK, LED_PIN);
		}

		// Poll USB
		usbd_poll(usbd_dev);

		if (dfu_state_ == STATE_DFU_MANIFEST) {
			// DFU complete!

			// Erase remaning pages (if any) on STM32F4
			#ifdef STM32F4
				erase_remaining_sectors();
			#endif

			// Hard reset
			system_hard_reset();
		}
	}
}