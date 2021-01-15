#ifndef __CONSTANTS_H__
    #define __CONSTANTS_H__ 

    #ifndef __CONFIG_H__
        #error "__CONSTANTS_H__" must be included after "__CONFIG_H__"
    #endif

    #include <libopencm3/stm32/memorymap.h>

    #ifdef STM32F0
        #define FLASH_BASE_ADDR			   0x08000000
        #define FLASH_BOOTLDR_SIZE_KB      10
        #define FLASH_BOOTLDR_SIZE_KB_INT  10
    #elif defined(STM32F1)
        #define FLASH_BASE_ADDR			   0x08000000
        #define FLASH_BOOTLDR_SIZE_KB      10
        #define FLASH_BOOTLDR_SIZE_KB_INT  10
    #elif defined(STM32F4)
        #define FLASH_BASE_ADDR			   0x08000000
        #define FLASH_BOOTLDR_SIZE_KB      16
        #define FLASH_BOOTLDR_SIZE_KB_INT  16
    #endif

    #ifdef ENABLE_AES_DECRYPTION
        #define CBC 1
        #define ECB 0
        #define CTR 0
    
        #include BOOT_KEY_FILE
    #endif

#endif