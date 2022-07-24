
#ifndef __UTILS_H
#define __UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <math.h>

/**
 * @brief Flash size register address
 */
#define ID_FLASH_ADDRESS (0x1FFF7A22)

/**
 * @brief Device ID register address
 */
#define ID_DBGMCU_IDCODE (0xE0042000)

/**
 * "Returns" the device signature
 *
 * Possible returns:
 *    - 0x0413: STM32F405xx/07xx and STM32F415xx/17xx)
 *    - 0x0419: STM32F42xxx and STM32F43xxx
 *    - 0x0423: STM32F401xB/C
 *    - 0x0433: STM32F401xD/E
 *    - 0x0431: STM32F411xC/E
 *
 * Returned data is in 16-bit mode, but only bits 11:0 are valid, bits 15:12 are always 0.
 * Defined as macro
 */
#define STM_ID_GetSignature() ((*(uint16_t *)(ID_DBGMCU_IDCODE)) & 0x0FFF)

/**
 * "Returns" the device revision
 *
 * Revisions possible:
 *    - 0x1000: Revision A
 *    - 0x1001: Revision Z
 *    - 0x1003: Revision Y
 *    - 0x1007: Revision 1
 *    - 0x2001: Revision 3
 *
 * Returned data is in 16-bit mode.
 */
#define STM_ID_GetRevision() (*(uint16_t *)(ID_DBGMCU_IDCODE + 2))

/**
* "Returns" the Flash size
*
* Returned data is in 16-bit mode, returned value is flash size in kB (kilo bytes).
*/
#define STM_ID_GetFlashSize() (*(uint16_t *)(ID_FLASH_ADDRESS))

#ifdef M_PI
#undef M_PI
#endif

#define HALF_M_PI   1.57079632679489661923f
#define M_PI        3.14159265358979323846f
#define TWO_M_PI    6.28318530717958647692f

#define RAD_TO_DEG  57.2957795130823208767f
#define DEG_TO_RAD  0.01745329251994329576f

#define MACRO_CONSTRAIN(amt, low, high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

#define MACRO_MAP(x, in_min, in_max, out_min, out_max)  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

#define MACRO_MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MACRO_MIN(x, y) (((x) < (y)) ? (x) : (y))

#define SQ(x) ((x) * (x))

static const float one_by_sqrt3 = 0.57735026919f;
static const float two_by_sqrt3 = 1.15470053838f;
static const float sqrt3_by_2 = 0.86602540378f;

//Beware of inserting large values!
static inline __attribute__((always_inline)) float wrap_pm(float x, float pm_range) {
    while (x >= pm_range) x -= (2.0f * pm_range);
    while (x < -pm_range) x += (2.0f * pm_range);
    return x;
}

//Beware of inserting large angles!
static inline __attribute__((always_inline)) float wrap_pm_pi(float theta) {
    while (theta >= M_PI) theta -= (TWO_M_PI);
    while (theta < -M_PI) theta += (TWO_M_PI);
    return theta;
}

static inline __attribute__((always_inline)) float wrap_pm_pi_large_angles(float theta) {
    theta = fmodf(theta, TWO_M_PI);
    if (theta >= M_PI) {
        theta -= TWO_M_PI;
    }
    return theta;
}

// like fmodf, but always positive
static inline __attribute__((always_inline)) float fmodf_pos(float x, float y) {
    float out = fmodf(x, y);
    if (out < 0.0f)
        out += y;
    return out;
}

// Map float from one range to another
/*
static inline __attribute__((always_inline)) float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}*/

// Compute rising edge timings (0.0 - 1.0) as a function of alpha-beta
// as per the magnitude invariant clarke transform
// The magnitude of the alpha-beta vector may not be larger than sqrt(3)/2
// Returns 0 on success, and -1 if the input was out of range
int SVM(float alpha, float beta, float* tA, float* tB, float* tC);

float fast_atan2(float y, float x);
float fast_sqrtf(float x);
float fast_absf(float x);
float fast_asinf(float x);
float fast_acosf(float x);

float horner_fma(float x, const float *coeffs, size_t count);
int mod(int dividend, int divisor);

uint32_t deadline_to_timeout(uint32_t deadline_ms);
uint32_t timeout_to_deadline(uint32_t timeout_ms);
int is_in_the_future(uint32_t time_ms);

uint32_t micros(void);
void delay_us(uint32_t us);

float our_arm_sin_f32(float x);
float our_arm_cos_f32(float x);

#ifdef __cplusplus
}
#endif

#endif  //__UTILS_H
