#ifndef __CONFIG_H__
    #define __CONFIG_H__

    /*
    ENABLE_FIRMWARE_UPLOAD: 
        Enables DFU upload commands, this is, enables reading
        flash memory (only within the user app boundaries) via DFU.

    ERASE_EXISTING_APPLICATION: 
        Ensures the user flash is completely erased before any
        DFU write/erase command is executed, to ensure no payloads are written
        that could lead to user data exfiltration.

    ENABLE_CHECKSUM: 
        Forces the user app image to have a valid checksum to
        boot it, on failure it will fallback to DFU mode.

    ENABLE_READOUT_PROTECTION_LEVEL_1: 
        Disables JTAG at startup before jumping to user code
        and also ensures RDP protection is enabled before booting. It will update
        option bytes if that is not met and force a reset (should only happen the
        first time, after that RDP is enabled and can only be disabled via JTAG).

    PROTECT_LAST_TWO_FLASH_PAGES:
        Prevents the last two pages of flash from being written to, which 
        might accidentally wipe out data we might have saved there with 
        bitsnap firmware.
    */

    // Bootloader Options
    #define ENABLE_FIRMWARE_UPLOAD
    #define ERASE_EXISTING_APPLICATION
    #define PROTECT_LAST_TWO_FLASH_PAGES
    //#define BOOT_KEY_FILE
    //#define ENABLE_READOUT_PROTECTION_LEVEL_1
    //#define ENABLE_CHECKSUM

    // Now in Makefile
    //#define I_MANUFACTURER "Civil Electric"
    //#define I_PRODUCT  "Bitsnap Bootloader"
    //#define ID_VENDOR  0x1337
    //#define ID_PRODUCT 0xC0DE

    #ifdef MTV
        #define LED_BANK              GPIOA
        #define LED_PIN               GPIO1

        // Use Boot1 PB2 as the button, as hardly anyone uses this pin as GPIO
        #define BUTTON_BANK           GPIOC
        #define BUTTON_PIN            GPIO2

	// LED Bank
    #elif defined(STM32F1)  
        //#define LED_BANK              GPIOA
        //#define LED_PIN               GPIO1
        
        // Blue Pill
        #define LED_BANK              GPIOC
        #define LED_PIN               GPIO13
        
        // Use Boot1 PB2 as the button, as hardly anyone uses this pin as GPIO
        #define BUTTON_BANK           GPIOB
        #define BUTTON_PIN            GPIO2

    #elif defined(STM32F4)
        
        // LD3 on F4 Discovery
        #define LED_BANK              GPIOA
        #define LED_PIN               GPIO1

        // User button on F4 Discovery
        #define BUTTON_BANK           GPIOB
        #define BUTTON_PIN            GPIO2
    #endif
    
    #include "bootloader_constants.h"

#endif