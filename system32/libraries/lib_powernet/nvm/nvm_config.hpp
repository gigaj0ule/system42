/*
* Convenience functions to load and store multiple objects from and to NVM.
* 
* The NVM stores consecutive one-to-one copies of arbitrary objects.
* The types of these objects are passed as template arguments to Config<Ts...>.
*/

/* Includes ------------------------------------------------------------------*/

#include <stdint.h>
#include <stdlib.h>

#include "nvm.h"
#include "crc.hpp"


/* Private defines -----------------------------------------------------------*/
#define CONFIG_CRC16_INIT 0xabcd
#define CONFIG_CRC16_POLYNOMIAL 0x3d65

#define CONFIG_CRC32_INIT 0x89ABCDEF
#define CONFIG_CRC32_POLYNOMIAL 0x04C11DB7

/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/
/* Private constant data -----------------------------------------------------*/

// IMPORTANT: if you change, reorder or otherwise modify any of the fields in
// the config structs, make sure to increment this number:
static constexpr uint16_t config_version = 0x0001;

/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Function implementations --------------------------------------------------*/


// @brief Manages configuration load and store operations from and to NVM
//
// The NVM stores consecutive one-to-one copies of arbitrary objects.
// The types of these objects are passed as template arguments to Config<Ts...>.
//
// Config<Ts...> has two template specializations to implement template recursion:
// - Config<T, Ts...> handles loading/storing of the first object (type T) and leaves
//   the rest of the objects to an "inner" class Config<Ts...>.
// - Config<> represents the leaf of the recursion.
template<typename ... Ts>
struct Config;

template<>
struct Config<> {
    static size_t get_size() {
        return 0;
    }
    static int load_config(size_t offset, uint32_t* crc) {
        return 0;
    }
    static int store_config(size_t offset, uint32_t* crc) {
        return 0;
    }
};

template<typename T, typename ... Ts>
struct Config<T, Ts...> {

    static size_t get_size() {
        return sizeof(T) + Config<Ts...>::get_size();
    }

    // @brief Loads one or more consecutive objects from the NVM.
    // During loading this function also calculates the CRC over the loaded data.
    // @param offset: 0 means that the function should start reading at the beginning
    // of the last comitted NVM block
    // @param crc: the result of the CRC calculation is written to this address
    // @param val0, vals: the values to be loaded
    static int load_config(size_t offset, uint32_t* crc, T* val0, Ts* ... vals) {

        size_t size = sizeof(T);

        // save current CRC (in case val0 and crc point to the same address)
        size_t previous_crc = *crc;
        if (NVM_read(offset, (uint8_t *)val0, size)) {
            return -1;
        }

        *crc = calc_crc16<CONFIG_CRC16_POLYNOMIAL>(previous_crc, (uint8_t *)val0, size);
        if (Config<Ts...>::load_config(offset + size, crc, vals...)) {
            return -1;
        }
        return 0;
    }

    // @brief Stores one or more consecutive objects to the NVM.
    // During storing this function also calculates the CRC over the stored data.
    // @param offset: 0 means that the function should start writing at the beginning
    // of the currently active NVM write block
    // @param crc: the result of the CRC calculation is written to this address
    // @param val0, vals: the values to be stored
    static int store_config(size_t offset, uint32_t* crc, const T* val0, const Ts* ... vals) {
        
        size_t size = sizeof(T);

        if (NVM_write(offset, (uint8_t *)val0, size)) {
            return -1;
        }
        // update CRC _after_ writing (in case val0 and crc point to the same address)
        if (crc) {
            *crc = calc_crc16<CONFIG_CRC16_POLYNOMIAL>(*crc, (uint8_t *)val0, size);
        }

        //if(vals) {
            if (Config<Ts...>::store_config(offset + size, crc, vals...)) {
                return -1;
            }
        //}
        return 0;
    }

    // @brief Loads one or more consecutive objects from the NVM. The loaded data
    // is validated using a CRC value that is stored at the beginning of the data.
    static int safe_load_config(T* val0, Ts* ... vals) {

        //printf("have %d bytes\r\n", NVM_get_max_read_length()); osDelay(5);
        volatile uint32_t readable_bitfields = Config<T, Ts..., uint32_t>::get_size();
        volatile uint32_t max_read_length = NVM_get_max_read_length();

        if (readable_bitfields > max_read_length) {
            return -1;
        }
        
        uint32_t crc_calculated = CONFIG_CRC16_INIT ^ config_version;
        uint32_t crc_stored;

        if (Config<T, Ts...>::load_config(0, &crc_calculated, val0, vals...)) {
            return -2;
        }

        NVM_read(Config<T, Ts...>::get_size(), (uint8_t *)&crc_stored, 4);

        if (crc_calculated != crc_stored) {
            return -3;
        }

        return 0;
    }

    // @brief Stores one or more consecutive objects to the NVM. In addition to the
    // provided objects, a CRC of the data is stored.
    //
    // The CRC includes a version number and thus adds some protection against
    // changes of the config structs during firmware update. Note that if the total
    // config data length changes, the CRC validation will fail even if the developer
    // forgets to update the config version number.
    static int safe_store_config(const T* val0, const Ts* ... vals) {

        size_t size = Config<T, Ts...>::get_size() + 4;
        
        if (size > NVM_get_max_write_length()) {
            return -1;
        }

        if (NVM_format(size)) {
            return -1;
        }
        
        uint32_t crc = CONFIG_CRC16_INIT ^ config_version;

        if (Config<T, Ts...>::store_config(0, &crc, val0, vals...)) {
            return -1;
        }

        NVM_write(Config<T, Ts...>::get_size(), (uint8_t *)&crc, 4);

        if (NVM_commit()) {
            return -1;
        }

        return 0;
    }
};
