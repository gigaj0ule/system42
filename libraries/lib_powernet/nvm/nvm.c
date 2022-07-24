/*
* Flash-based Non-Volatile Memory (NVM)
* 
* This file supports storing and loading persistent configuration based on
* the STM32 builtin flash memory.
*
* The STM32F405xx has 12 flash sectors of heterogeneous size. We use the last
* two sectors for configuration data. These pages have a size of 128kB each.
* Setting any bit in these sectors to 0 is always possible, but setting them
* to 1 requires erasing the whole sector.
*
* We consider each sector as an array of 64-bit fields except the first N bytes, which we
* instead use as an allocation block. The allocation block is a compact bit-field (2 bit per entry)
* that keeps track of the state of each field (erased, invalid, valid).
*
* One sector is always considered the valid (read) sector and the other one is the
* target_sector for the next write access: they can be considered to be ping-pong or double buffred.
*
* When writing a block of data, instead of always erasing the whole writable sector the
* new data is appended in the erased area. This presumably increases flash life span.
* The writable sector is only erased if there is not enough space for the new data.
*
* On startup, if there is exactly one sector
* whose last non-erased value has the state "valid" that sector is considered
* the valid sector. In any other case the selection is undefined.
*
*
* To write a new block of data atomically we first mark all associated fields
* as "invalid" (in the allocation table) then write the data and then mark the
* fields as "valid" (in the direction of increasing address).
*/

#include "nvm.h"
#include <string.h>

#if defined(STM32F1xx)
    #include <stm32f1xx.h>
    #include <stm32f1xx_hal.h>
    #include <stm32f1xx_hal_flash.h>
    #include <stm32f1xx_hal_flash_ex.h>

    // We now calculate all of this at runtime for the F103
    // so we can support different flash sizes so these values
    // will be over-written

    #define FLASH_SECTOR_A_SIZE FLASH_PAGE_SIZE
    #define FLASH_SECTOR_B_SIZE FLASH_PAGE_SIZE
    #define FLASH_SECTOR_A_BASE (const volatile uint8_t*)(0x8000000UL + 126UL * FLASH_PAGE_SIZE)
    #define FLASH_SECTOR_B_BASE (const volatile uint8_t*)(0x8000000UL + 127UL * FLASH_PAGE_SIZE)
    #define FLASH_SECTOR_A (0x8000000UL + 126UL * FLASH_SECTOR_A_SIZE)
    #define FLASH_SECTOR_B (0x8000000UL + 127UL * FLASH_SECTOR_B_SIZE)

    //#define HAL_FLASH_ClearError() __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR)
    #define HAL_FLASH_ClearError()

#elif defined(STM32F405xx)

    #include <stm32f405xx.h>
    #include <stm32f4xx_hal.h>

    #define FLASH_SECTOR_A FLASH_SECTOR_10
    #define FLASH_SECTOR_B FLASH_SECTOR_11

    // refer to page 75 of datasheet:
    // http://www.st.com/content/ccc/resource/technical/document/reference_manual/3d/6d/5a/66/b4/99/40/d4/DM00031020.pdf/files/DM00031020.pdf/jcr:content/translations/en.DM00031020.pdf
    #define FLASH_SECTOR_A_BASE (const volatile uint8_t*)0x80C0000UL
    #define FLASH_SECTOR_A_SIZE 0x20000UL
    #define FLASH_SECTOR_B_BASE (const volatile uint8_t*)0x80E0000UL
    #define FLASH_SECTOR_B_SIZE 0x20000UL

    #define HAL_FLASH_ClearError() __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR)

#elif defined(STM32F407xx)

    // @todo: Likely wrong

    #include <stm32f407xx.h>
    #include <stm32f4xx_hal.h>

    #define FLASH_SECTOR_A FLASH_SECTOR_10
    #define FLASH_SECTOR_B FLASH_SECTOR_11

    // refer to page 75 of datasheet:
    // http://www.st.com/content/ccc/resource/technical/document/reference_manual/3d/6d/5a/66/b4/99/40/d4/DM00031020.pdf/files/DM00031020.pdf/jcr:content/translations/en.DM00031020.pdf
    #define FLASH_SECTOR_A_BASE (const volatile uint8_t*)0x80C0000UL
    #define FLASH_SECTOR_A_SIZE 0x20000UL
    #define FLASH_SECTOR_B_BASE (const volatile uint8_t*)0x80E0000UL
    #define FLASH_SECTOR_B_SIZE 0x20000UL

    #define HAL_FLASH_ClearError() __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR)

#else

#error "No NVM config yet for specified microcontroller"

#endif


typedef enum {
    VALID = 0,
    INVALID = 1,
    ERASED = 3
} field_state_t;

typedef struct {
    size_t max_valid_data_index;
    uint32_t sector_id;   //!< HAL ID of this sector
    size_t n_data;        //!< number of 64-bit fields in this sector
    size_t n_reserved;    //!< number of 64-bit fields in this sector that are reserved for the allocation table
    const volatile uint8_t* alloc_table;
    const volatile uint64_t* data;
} sector_t;

sector_t sectors[] = { 
    {
        .sector_id = FLASH_SECTOR_A,
        .n_data = FLASH_SECTOR_A_SIZE >> 3,
        .n_reserved = (FLASH_SECTOR_A_SIZE >> 3) >> 5,
        .alloc_table = FLASH_SECTOR_A_BASE,
        .data = (uint64_t *)FLASH_SECTOR_A_BASE
    },
    {
        .sector_id = FLASH_SECTOR_B,
        .n_data = FLASH_SECTOR_B_SIZE >> 3,
        .n_reserved = (FLASH_SECTOR_B_SIZE >> 3) >> 5,
        .alloc_table = FLASH_SECTOR_B_BASE,
        .data = (uint64_t *)FLASH_SECTOR_B_BASE
    }
};

uint8_t read_sector_;      // 0 or 1 to indicate which sector to read from and which to write to
uint8_t write_sector_;
size_t writable_bitfields_;  // number of 64-bit values that were reserved using NVM_format
size_t readable_bitfields_;  // number of 64-bit fields that can be read

// @brief Erases a flash sector. This sets all bits in the sector to 1.
// The sector's current index is reset to the minimum value (n_reserved).
// @returns 0 on success or a non-zero error code otherwise
int erase(sector_t *target_sector) {

    #if defined(STM32F405xx) || defined(STM32F407xx)
    FLASH_EraseInitTypeDef erase_struct = {
        .TypeErase = FLASH_TYPEERASE_SECTORS,
        .Banks = 0, // only used for mass erase
        .Sector = target_sector->sector_id,
        .NbSectors = 1,
        .VoltageRange = FLASH_VOLTAGE_RANGE_3
    };
    #elif defined(STM32F1xx)
    FLASH_EraseInitTypeDef erase_struct = {
        .TypeErase = FLASH_TYPEERASE_PAGES, 
        .Banks = 0,
        .PageAddress = target_sector->sector_id,
        .NbPages = 1
    };
    #endif

    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();
    uint32_t sector_error;

    if (HAL_FLASHEx_Erase(&erase_struct, &sector_error) != HAL_OK) {
        goto fail;
    }

    #if defined(STM32F1xx)
    FLASH_WaitForLastOperation(0);
    #endif

    //target_sector->index = target_sector->n_reserved;

    HAL_FLASH_Lock();
    return 0;

    fail: {
        HAL_FLASH_Lock();
        //printf("erase failed: %u \r\n", HAL_FLASH_GetError());
        return HAL_FLASH_GetError(); // non-zero
    }
}


// @brief Writes states into the allocation table.
// The write operation goes in the direction of increasing indices.
// @param state: 11: erased, 10: writing, 00: valid data
// @returns 0 on success or a non-zero error code otherwise
int set_allocation_state(sector_t *target_sector, size_t count, field_state_t state) {

    // expand state to state for 4 values
    const uint8_t states = (state << 0) | (state << 2) | (state << 4) | (state << 6);    
    
    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();

    uint32_t state_word = (states << 24) + (states << 16) + (states << 8) + states; 

    // write states
    for( int i = 0; i < count; i+= 4){
        if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (uintptr_t)&target_sector->alloc_table[i], state_word) != HAL_OK) {   
            goto fail;
        }  
    }

    HAL_FLASH_Lock();
    return 0;

    fail: {
        HAL_FLASH_Lock();
        return HAL_FLASH_GetError(); // non-zero
    }
}

// @brief Reads the allocation table from behind to determine how many fields match the
// reference state.
// @param sector: The sector on which to perform the search
// @param max_index: The maximum index that should be considered
// @param ref_state: The reference state
// @param state: Set to the first encountered state that is unequal to ref_state.
//               Set to ref_state if all encountered states are equal to ref_state.
// @returns The smallest index that points to a field with ref_state.
//          This value is at least sector->n_reserved and at most max_index.
size_t scan_allocation_table(sector_t *target_sector, size_t max_index, field_state_t ref_state, field_state_t *state) {

    const uint8_t ref_states = (ref_state << 0) | (ref_state << 2) | (ref_state << 4) | (ref_state << 6);
    size_t index = (((max_index + 3) >> 2) << 2);
    uint8_t states = ref_states;

    for (; index >= (target_sector->n_reserved + 4); index -= 4) {

        states = target_sector->alloc_table[(index - 1) >> 2];
        
        if (states != ref_states) {
            break;
        }
    }   

    // once we encounterd a byte with any state mismatch determine which of the 4 states it is
    for (; ((states >> 6) == (ref_states & 0x3)) && (index > target_sector->n_reserved); index--) {
        states <<= 2;
    }
    
    *state = states >> 6;

    return index;
}

#if defined(STM32F1xx)
    // =========================================================================
    // Returns flash size (in kilobytes)
    #define FLASH_SIZE_REG 0x1FFFF7E0

    static uint32_t getFlashSize(void) {
        // From address register	
        uint32_t *flashSize = (uint32_t *) (FLASH_SIZE_REG);
        volatile uint32_t flash_size = (*flashSize & 0xffff) * 1024;
        return flash_size;
    }

    // =========================================================================
    // Returns flash page size
    static uint16_t getFlashPageSize(void) {

        uint32_t flash_size = getFlashSize();
        
        if (flash_size > (128 * 1024)) {
            // STM Datasheet says 2kB for big devices
            return 0x800;
        }
        else {
            // 1kB for small ones
            return 0x400;
        }
    }
#endif

// Loads the head of the NVM data.
// If this function fails subsequent calls to NVM functions (other than NVM_init or NVM_erase)
// cause undefined behavior.
// @returns 0 on success or a non-zero error code otherwise
int NVM_init(void) {

    #if defined(STM32F1xx) 
        uint32_t flash_size = getFlashSize();
        uint32_t flash_end = 0x8000000UL + flash_size;
        uint32_t flash_page_size = getFlashPageSize();

        uint32_t flash_sector_a_base = (flash_end - (2 * flash_page_size));//- VECT_TAB_OFFSET;

        sectors[0].sector_id = flash_sector_a_base;
        sectors[0].n_data = flash_page_size >> 3;
        sectors[0].n_reserved = (flash_page_size >> 3) >> 5;
        sectors[0].alloc_table = (uint8_t*)flash_sector_a_base;
        sectors[0].data = (uint64_t *)flash_sector_a_base;
    
        uint32_t flash_sector_b_base = (flash_end - flash_page_size);//- VECT_TAB_OFFSET;

        sectors[1].sector_id = flash_sector_b_base;
        sectors[1].n_data = flash_page_size >> 3;
        sectors[1].n_reserved = (flash_page_size >> 3) >> 5;
        sectors[1].alloc_table = (uint8_t*)flash_sector_b_base;
        sectors[1].data = (uint64_t *)flash_sector_b_base;
    #endif

    field_state_t sector0_state, sector1_state;

    sectors[0].max_valid_data_index = scan_allocation_table(&sectors[0], sectors[0].n_data, ERASED, &sector0_state);
    sectors[1].max_valid_data_index = scan_allocation_table(&sectors[1], sectors[1].n_data, ERASED, &sector1_state);

    // Select valid sector on a best effort basis
    // (in unfortunate cases valid_sector might actually point
    // to an invalid or erased sector)
    read_sector_ = 0;
    write_sector_ = 1;

    if (sector1_state == VALID) {
        read_sector_ = 1;
        write_sector_ = 0;
    }
       
    // count the number of valid fields
    sector_t *target_sector = &sectors[read_sector_];

    uint8_t first_nonvalid_state;

    volatile uint32_t min_valid_index = scan_allocation_table(target_sector, target_sector->max_valid_data_index, VALID, &first_nonvalid_state);
    
    readable_bitfields_ = target_sector->max_valid_data_index - min_valid_index;
    writable_bitfields_ = 0;

    int status = 0;
    
    /*
    // bring non-valid sectors into a known state
    this is not absolutely required
    if (sector0_state != VALID) {
        status |= erase(&sectors[0]);
    }
    if (sector1_state != VALID) {
        status |= erase(&sectors[1]);
    }
    */
    
    return status;
}

// @brief Erases all data in the NVM.
//
// If this function fails subsequent calls to NVM functions (other than NVM_init or NVM_erase)
// cause undefined behavior.
// Caution: this function may take a long time (like 1 second)
//
// @returns 0 on success or a non-zero error code otherwise
int NVM_erase(void) {

    read_sector_ = 0;
    write_sector_ = 1;

    sectors[0].max_valid_data_index = sectors[0].n_reserved;
    sectors[1].max_valid_data_index = sectors[1].n_reserved;

    int state = 0;
    state |= erase(&sectors[0]);
    state |= erase(&sectors[1]);
    return state;
}

// @brief Returns the maximum number of bytes that can be read using NVM_read.
// This holds until NVM_commit is called.
size_t NVM_get_max_read_length(void) {
    return readable_bitfields_ << 3;
}

// @brief Returns the maximum length (in bytes) that can passed to NVM_format.
// This holds until NVM_commit is called.
size_t NVM_get_max_write_length(void) {
    
    sector_t *target_sector = &sectors[write_sector_];
    return (target_sector->n_data - target_sector->n_reserved) << 3;
}

// @brief Reads from the latest committed block in the non-volatile memory.
// @param offset: offset in bytes (0 meaning the beginning of the valid area)
// @param data: buffer to write to
// @param length: length in bytes (if (offset + length) is out of range, the function fails)
// @returns 0 on success or a non-zero error code otherwise
int NVM_read(size_t offset, uint8_t *data, size_t length) {

    sector_t *target_sector = &sectors[read_sector_];
    const uint8_t *src_ptr = ((const uint8_t *)&target_sector->data[target_sector->max_valid_data_index - readable_bitfields_]) + offset;
    memcpy(data, src_ptr, length);
    return 0;
}

// @brief Prepares an NVM write operation.
//
// The most recent valid NVM data is not modified or invalidated until NVM_commit is called.
// The length must be at most equal to the size indicated by NVM_get_max_write_length().
//
// @param length: Length of the staging block that should be created
int NVM_format(size_t length) {

    int status = 0;

    sector_t *target_sector = &sectors[write_sector_];

    length = (length + 7) >> 3; // round to multiple of 64 bit
    
    if ((status = erase(target_sector))) {
        return status;
    }

    writable_bitfields_ = length;

    return 0;
}

// @brief Writes to the current data block that was opened with NVM_format.
//
// The operation fails if (offset + length) is larger than the length passed to NVM_format.
// The most recent valid NVM data is not modified or invalidated until NVM_commit is called.
// Warning: Writing different data to the same area multiple times during a single transaction
// will cause data corruption.
//
// @param offset: The offset in bytes, 0 being the beginning of the staging block.
// @param data: Pointer to the data that should be written
// @param length: Data length in bytes
int NVM_write(size_t offset, uint8_t *data, size_t length) {

    sector_t *target_sector = &sectors[write_sector_];

    HAL_FLASH_Unlock();
    HAL_FLASH_ClearError();
    
    // write 32-bit values (64-bit doesn't work)
    for (; length >= 4; data += 4, offset += 4, length -=4) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                ((uintptr_t)&target_sector->data[target_sector->n_reserved]) + offset, *(uint32_t*)data) != HAL_OK) {
            goto fail;
        }
    }

    HAL_FLASH_Lock();
    return 0;
fail:
    HAL_FLASH_Lock();
    return HAL_FLASH_GetError(); // non-zero
}

// @brief Commits the new data to NVM atomically.
int NVM_commit(void) {

    sector_t *target_sector = &sectors[write_sector_];

    // mark the newly-written fields as valid
    int status = set_allocation_state(target_sector, writable_bitfields_, VALID);

    if (status) {
        return status;
    }
    
    // Blow out old sector
    erase(&sectors[read_sector_]);

    write_sector_ = read_sector_;
    read_sector_ = 1 - read_sector_;

    readable_bitfields_ = writable_bitfields_;
    writable_bitfields_ = 0;

    return status;
}




#ifdef INCLUDE_NVM_DEMO 
#include <cmsis_os.h>
/** @brief Call this at startup to test/demo the NVM driver

 Expected output when starting with a fully erased NVM

    [1st boot]
    === NVM TEST ===
    NVM is empty
    write 0x00, ..., 0x25 to NVM
    new data committed to NVM
    
    [2nd boot]
    === NVM TEST ===
    NVM contains 40 valid bytes:
    00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f
    10 11 12 13 14 15 16 17 18 19 1a 1b 1c 1d 1e 1f
    20 21 22 23 24 25 ff ff
    write 0xbd, ..., 0xe2 to NVM
    new data committed to NVM

    [3rd boot]
    === NVM TEST ===
    NVM contains 40 valid bytes:
    bd be bf c0 c1 c2 c3 c4 c5 c6 c7 c8 c9 ca cb cc
    cd ce cf d0 d1 d2 d3 d4 d5 d6 d7 d8 d9 da db dc
    dd de df e0 e1 e2 ff ff
    write 0xcb, ..., 0xf0 to NVM
    new data committed to NVM
*/
void NVM_demo(void) {
    const size_t len = 38;
    uint8_t data[len];
    int progress = 0;
    uint8_t seed = 0;

    osDelay(100);
    printf("=== NVM TEST ===\r\n"); osDelay(5);
    //NVM_erase();
    if (progress++, NVM_init() != 0)
        goto fail;
    
    // load bytes from NVM and print them
    size_t available = NVM_get_max_read_length();
    if (available) {
        printf("NVM contains %d valid bytes:\r\n", available); osDelay(5);
        uint8_t buf[available];
        if (progress++, NVM_read(0, buf, available) != 0) {
            goto fail;
        }
        for (size_t pos = 0; pos < available; ++pos) {
            seed += buf[pos];
            printf(" %02x", buf[pos]);
            if ((((pos + 1) % 16) == 0) || ((pos + 1) == available)) {
                printf("\r\n");
            }
            osDelay(2);
        }
    } 
    else {
        printf("NVM is empty\r\n"); 
        osDelay(5);
    }

    // store new bytes in NVM (data based on seed)
    printf("write 0x%02x, ..., 0x%02x to NVM\r\n", seed, seed + len - 1); osDelay(5);
    for (size_t i = 0; i < len; i++) {
        data[i] = seed++;
    }
    if (progress++, NVM_format(len) != 0) {
        goto fail;
    }
    if (progress++, NVM_write(0, data, len / 2)) {
        goto fail;
    }
    if (progress++, NVM_write(len / 2, &data[len / 2], len - (len / 2))) {
        goto fail;
    }
    if (progress++, NVM_commit()) {
        goto fail;
    }
    printf("new data committed to NVM\r\n"); osDelay(5);

    return;

fail:
    printf("NVM test failed at %d!\r\n", progress);
}
#endif