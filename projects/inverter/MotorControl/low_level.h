/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOW_LEVEL_H
#define __LOW_LEVEL_H

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include <stdbool.h>
#include <adc.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define ADC_CHANNEL_COUNT 16

/* Exported variables --------------------------------------------------------*/
extern float vbus_voltage;
extern bool brake_resistor_armed;
extern uint16_t adc_measurements_[ADC_CHANNEL_COUNT];

/* Exported macro ------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

void safety_critical_arm_motor_pwm(Motor& motor);
bool safety_critical_disarm_motor_pwm(Motor& motor);
void safety_critical_arm_brake_resistor();
void safety_critical_disarm_brake_resistor();
void safety_critical_motor_pwm_timings_state_machine(Motor& motor);

bool is_endpoint_ref_valid(endpoint_ref_t endpoint_ref);

// called from STM platform code
extern "C" {
    void pwm_input_capture_callback(int channel, uint32_t timestamp);
}

// Initalization
void start_adc_pwm();
void start_pwm(TIM_HandleTypeDef* htim);
void sync_timers(TIM_HandleTypeDef* htim_a, TIM_HandleTypeDef* htim_b,
                 uint16_t TIM_CLOCKSOURCE_ITRx, uint16_t count_offset,
                 TIM_HandleTypeDef* htim_refbase = nullptr);
void start_general_purpose_adc();
void start_timer_ccr_dma();
void start_analog_thread();
void start_pwm_input_capture();

// Read, Write
float get_adc_voltage(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin);
int16_t get_adc_value(GPIO_TypeDef* GPIO_port, uint16_t GPIO_pin);

inline uint32_t cpu_enter_critical() {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

inline void cpu_exit_critical(uint32_t priority_mask) {
    __set_PRIMASK(priority_mask);
}

#ifdef __cplusplus
}
#endif



#endif //__LOW_LEVEL_H
