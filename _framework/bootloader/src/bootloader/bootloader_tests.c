#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/tools.h>

#include "bootloader_config.h"
#include "bootloader_tests.h"
#include "bootloader_utils.h"
#include "bootloader_flash_functions.h"

static const char hextochar[16] = "0123456789ABCDEF";


// =========================================================================
uint64_t read_device_serial_number() {
    // This procedure of building a USB serial number should be identical
    // to the way the STM's built-in USB bootloader does it. This means
    // that the device will have the same serial number in normal and DFU mode.
    uint32_t uuid[3];

    desig_get_unique_id(uuid);

    uint32_t uuid_mixed_part = uuid[0] + uuid[2];
    uint64_t serial_number = ((uint64_t)uuid_mixed_part << 16) | (uint64_t)(uuid[1] >> 16);
    return serial_number;
}


// =========================================================================
void print_device_serial_number(char * s) {

    uint64_t val = read_device_serial_number();

    for (uint8_t i = 0; i < 12; ++i) {
        s[i] = hextochar[(val >> (48-4)) & 0xf];
        val <<= 4;
    }
}


// =========================================================================
// Checks if S/N is at end of bootloader flash. 
// If it is not, then it attempts to save it to the end of the flash.
int8_t check_set_device_unique() {

    // Get device serial number
    uint64_t serial_number = read_device_serial_number();

    // Get test address
    uint32_t storage_address =  (FLASH_BASE_ADDR + (1024 * FLASH_BOOTLDR_SIZE_KB_INT) - sizeof(serial_number));
    uint64_t * end_of_flash_ptr = (uint64_t * ) storage_address;

    // Invert serial number
    uint64_t not_serial_number = ~serial_number;
    uint64_t saved_token = end_of_flash_ptr[0];

    // What is the token?
    if(saved_token == 0xFFFFFFFFFFFFFFFF) {
        // If there is no saved SN we should save the token now
        flash_unlock();
        _flash_program_buffer(storage_address, (uint8_t * ) &not_serial_number, sizeof(not_serial_number));
        flash_lock();
        flash_wait_for_last_operation();

        // Then reboot
        system_hard_reset();
    }
    else if (saved_token == not_serial_number) {
        // If there is a token and it matches the S/N then the device is probably genuine
        return 1;
    }
    else {
        // If there is a token but it is not the S/N the device is probably a clone
        return 0;
    }
    
    return -1;
}


// =========================================================================
#if defined(ENABLE_READOUT_PROTECTION_LEVEL_1)
    void check_set_rdp(){
        /*
        Bits [31:24] nUSER
        Bits [23:16] USER: User option byte (stored in FLASH_OBR[9:2])
            This byte is used to configure the following features:
            – Select the watchdog event: Hardware or software.
            – Reset event when entering Stop mode.
            – Reset event when entering Standby mode.
            Note: Only bits [16:18] are used, bits [23:19]: 0x1F are not used.
        Bit 18: nRST_STDBY
            0: Reset generated when entering Standby mode.
            1: No reset generated.
        Bit 17: nRST_STOP
            0: Reset generated when entering Stop mode
            1: No reset generated
        Bit 16: WDG_SW
            0: Hardware watchdog
            1: Software watchdog
        Bits [15:8]: nRDP
        Bits [7:0]: RDP: Read protection option byte
            The read protection helps the user protect the software code stored in Flash
            memory. It is activated by setting the RDP option byte.
            When this option byte is programmed to a correct value (RDPRT key = 0x00A5),
            read access to the Flash memory is allowed.
            (The result of RDP level enabled/disabled is stored in FLASH_OBR[1].)
        */

        // Check for RDP protection, and in case it's not enabled, do it!
        #if defined(STM32F1)
        if (!(FLASH_OBR & 0x2)) {
            // Read protection NOT enabled ->
            
            // Unlock option bytes
            flash_unlock();
            flash_unlock_option_bytes();

            // Delete them all
            flash_erase_option_bytes();

            // Now write a pair of bytes that are complentary [RDP, nRDP]
            //flash_program_option_bytes(INFO_BASE + 0x800, 0x5AA5); //(unlock)
            flash_program_option_bytes(INFO_BASE + 0x800, 0x33CC); //(lock)

            // Wait for option bytes to finish
            flash_wait_for_last_operation();

            // Lock flash
            flash_lock();

            // Now reset, for RDP to take effect. 
            // We should not re-enter this path
            system_hard_reset();
        }
        #endif

        // Disable JTAG and SWD to prevent debugging/readout
        // Well this is not actually needed is it?
        // RDP should take care of this
        #if defined(STM32F1)
            //gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO13);
            //gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO14);
        #elif defined(STM32F4)
            #warning todo: implement swd disable on f4!
        #endif
    }
#endif

// =========================================================================
bool test_button_state() {
    return gpio_get(BUTTON_BANK, BUTTON_PIN);
}

// =========================================================================
static inline int getFlashEnd(void) {
    // From address register
    uint16_t flashSize = desig_get_flash_size();
    return flashSize * 1024 + FLASH_BASE_ADDR;
}

// =========================================================================
bool dfu_boot_token() {
    // If we find a valid boot token in RAM, the application is asking us explicitly
    // to enter DFU mode. This is used to implement the DFU_DETACH command when the app
    // is running.
    uint32_t * user_code_boot_token = (uint32_t *)(0x20005000 - 4);

    if(*user_code_boot_token == 0xB105F00D) {
        user_code_boot_token[0] = 0;
        return true;
    }
    else {
        return false;
    }
}

// =========================================================================
bool test_if_user_app_is_missing(uint32_t test_address) {
    // If there doesn't seem to be a valid application installed, we always go to
    // bootloader mode.

    uint32_t * user_code_ivt = (uint32_t *)test_address;

	// Return true if the contents of 1st address of the vector table (which is the stack pointer)
	// point to a valid RAM address, and if the 2nd address of the IVT (which is the program counter) 
    // points to an address outside the application flash area (eg, 0xFFFFFFFF).
    bool user_application_is_missing = false;

    // Stack pointer
    volatile uint32_t eStack = user_code_ivt[0];
    if(eStack < 0x20000000 || eStack > 0x30000000) {
        user_application_is_missing = true;
    }

    // Program Counter
    volatile uint32_t pc = user_code_ivt[1];
    if(pc < test_address || pc >= getFlashEnd()) {
        user_application_is_missing = true;
    }

    return user_application_is_missing;
}
