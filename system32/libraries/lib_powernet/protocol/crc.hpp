#ifndef __CRC_HPP
#define __CRC_HPP

#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <type_traits>


// Calculates an arbitrary CRC for one byte.
// Adapted from https://barrgroup.com/Embedded-Systems/How-To/CRC-Calculation-C-Code
// and http://mercury.pr.erau.edu/~siewerts/cec450/code/example-3/crc.c

template<typename T, unsigned POLYNOMIAL>
static T calc_crc(T remainder, uint8_t value, bool endian) {
    constexpr T BIT_WIDTH = (CHAR_BIT * sizeof(T));
    constexpr T TOPBIT = ((T)1 << (BIT_WIDTH - 1));
    
    if (endian == false) {
        // Little endian mode
        remainder ^= value;

		for (uint8_t bit = 0; bit < 8; bit++) {

            T multiple;

            if(remainder & 1) {
                multiple = POLYNOMIAL;
            }
            else {
                multiple = 0;
            }

            remainder = (remainder >> 1) ^ multiple;
        }
    }
    else {
        // Big endian mode

        // Bring the next byte into the remainder.
        remainder ^= (value << (BIT_WIDTH - 8));

        // Perform modulo-2 division, a bit at a time.
        for (uint8_t bit = 8; bit; --bit) {
            if (remainder & TOPBIT) {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            } else {
                remainder = (remainder << 1);
            }
        }
    }

    return remainder;
}


uint32_t calc_crc32_fast(uint32_t remainder, uint8_t value, bool endian);

template<typename T, unsigned POLYNOMIAL>
static T calc_crc_buf(T remainder, const uint8_t* buffer, size_t length, bool endian=true, bool fast=false) {

    // Disabled seeing as it did not actually help make things faster...
    if(fast){
        while (length--) {
            remainder = calc_crc32_fast(remainder, *(buffer++), endian);
        }
    }
    else {
        while (length--) {
            remainder = calc_crc<T, POLYNOMIAL>(remainder, *(buffer++), endian);
        }
    }

    return remainder;
}

template<unsigned POLYNOMIAL>
static uint8_t calc_crc8(uint8_t remainder, uint8_t value, bool endian=true) {
    return calc_crc<uint8_t, POLYNOMIAL>(remainder, value, endian);
}

template<unsigned POLYNOMIAL>
static uint16_t calc_crc16(uint16_t remainder, uint8_t value, bool endian=true) {
    return calc_crc<uint16_t, POLYNOMIAL>(remainder, value, endian);
}

template<unsigned POLYNOMIAL>
static uint32_t calc_crc32(uint32_t remainder, uint8_t value, bool endian=true) {
    return calc_crc<uint32_t, POLYNOMIAL>(remainder, value, endian);
}

template<unsigned POLYNOMIAL>
static uint8_t calc_crc8(uint8_t remainder, const uint8_t* buffer, size_t length, bool endian=true) {
    return calc_crc_buf<uint8_t, POLYNOMIAL>(remainder, buffer, length, endian);
}

template<unsigned POLYNOMIAL>
static uint16_t calc_crc16(uint16_t remainder, const uint8_t* buffer, size_t length, bool endian=true) {
    return calc_crc_buf<uint16_t, POLYNOMIAL>(remainder, buffer, length, endian);
}

template<unsigned POLYNOMIAL>
static uint32_t calc_crc32(uint32_t remainder, const uint8_t* buffer, size_t length, bool endian=true, bool fast = false) {
    return calc_crc_buf<uint32_t, POLYNOMIAL>(remainder, buffer, length, endian, fast);
}

#endif /* __CRC_HPP */
