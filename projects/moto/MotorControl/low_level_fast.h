/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOW_LEVEL_FAST_H
#define __LOW_LEVEL_FAST_H

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <cmsis_os.h>
#include <stdbool.h>

// STM32 Callbacks
void ADC_IRQHandler(void);
void TIM1_UP_TIM10_IRQHandler();
void TIM8_UP_TIM13_IRQHandler();

// Motor functions
void update_brake_current();
void pwm_state_machine(bool injected, uint16_t now);
void vbus_voltage_sense_calculation();
void safety_critical_apply_brake_resistor_timings(uint32_t low_off, uint32_t high_on);
void check_if_run_control_loop_failed(Motor * axis_motor);

#ifdef __cplusplus
}
#endif



#endif