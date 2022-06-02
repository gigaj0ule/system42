/**
* @file lp8545.h
*
* @brief SM72242 Driver _H
*
* @copyright
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation; either
* version 3.0 of the License, or (at your option) any later version.
*
* @copyright
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*
* @author Arduino LLC
* @author Adam Munich
*/
 
#ifndef _SM72242_H_
	#define _SM72242_H_

	#include <Arduino.h>
	#include "Wire.h"

	//#############################################################################
	// Macros
	//-----------------------------------------------------------------------------
	// Device address
	#define SM_DEFAULT_I2C_ADDRESS			0x01

	// ----------------------------------------------------------------------------------
	// Register IDs
	#define SM_REGISTER_REG_0				224
	#define SM_REGISTER_REG_1				225
	#define SM_REGISTER_REG_2				226
	#define SM_REGISTER_REG_3				227
	#define SM_REGISTER_REG_4				228
	#define SM_REGISTER_REG_5				229

	#define SM_REGISTER_LENGTH_BYTES		8

	// ----------------------------------------------------------------------------------
	// REG0
	#define SM_BITMASK_ADC0					0b1111111111
	#define SM_BITSHIFT_ADC0				0

	#define SM_BITMASK_ADC2					0b1111111111
	#define SM_BITSHIFT_ADC2				10

	#define SM_BITMASK_ADC4					0b1111111111
	#define SM_BITSHIFT_ADC4				20

	#define SM_BITMASK_ADC6					0b1111111111
	#define SM_BITSHIFT_ADC6				30

	// REG1
	#define SM_BITMASK_IIN					0b1111111111
	#define SM_BITSHIFT_IIN					0

	#define SM_BITMASK_VIN					0b1111111111
	#define SM_BITSHIFT_VIN					10

	#define SM_BITMASK_IOUT					0b1111111111
	#define SM_BITSHIFT_IOUT				20

	#define SM_BITMASK_VOUT					0b1111111111
	#define SM_BITSHIFT_VOUT 				30

	// REG3 
	#define SM_BITMASK_OPEN_LOOP			0b1
	#define SM_BITSHIFT_OPEN_LOOP			0

	#define SM_BITMASK_PLL_CLOCK			0b1
	#define SM_BITSHIFT_PLL_CLOCK			1

	#define SM_BITMASK_RESET				0b1
	#define SM_BITSHIFT_RESET				2

	#define SM_BITMASK_PASS_THRU_MAN		0b1
	#define SM_BITSHIFT_PASS_THRU_MAN		3

	#define SM_BITMASK_PASS_THRU_SEL		0b1
	#define SM_BITSHIFT_PASS_THRU_SEL		4

	#define SM_BITMASK_DUTY_CYCLE_OPEN		0b111111111
	#define SM_BITSHIFT_DUTY_CYCLE_OPEN 	5

	#define SM_BITMASK_DEADTIME_ON			0b111
	#define SM_BITSHIFT_DEADTIME_ON			14

	#define SM_BITMASK_DEADTIME_OFF			0b111
	#define SM_BITSHIFT_DEADTIME_OFF		17

	#define SM_BITMASK_VOUT_MAX				0b1111111111
	#define SM_BITSHIFT_VOUT_MAX			20

	#define SM_BITMASK_IOUT_MAX				0b1111111111
	#define SM_BITSHIFT_IOUT_MAX			30

	#define SM_BITMASK_BB_IN_PT_MODE_SEL	0b11
	#define SM_BITSHIFT_BB_IN_PT_MODE_SEL	40

	#define SM_BITMASK_POWER_THR_SEL		0b1
	#define SM_BITSHIFT_POWER_THR_SEL		42

	#define SM_BITMASK_OVERRIDE_ADCPROG		0b1
	#define SM_BITSHIFT_OVERRIDE_ADCPROG	46


	// REG4
	#define SM_BITMASK_IIN_OFFSET			0b11111111
	#define SM_BITSHIFT_IIN_OFFSET			0

	#define SM_BITMASK_VIN_OFFSET			0b11111111
	#define SM_BITSHIFT_VIN_OFFSET			8

	#define SM_BITMASK_IOUT_OFFSET			0b11111111
	#define SM_BITSHIFT_IOUT_OFFSET			16

	#define SM_BITMASK_VOUT_OFFSET			0b11111111
	#define SM_BITSHIFT_VOUT_OFFSET			24

	// REG5
	#define SM_BITMASK_IOUT_LO_TH			0b1111111111
	#define SM_BITSHIFT_IOUT_LO_TH			0

	#define SM_BITMASK_IOUT_HI_TH			0b1111111111
	#define SM_BITSHIFT_IOUT_HI_TH			10

	#define SM_BITMASK_IIN_LO_TH			0b1111111111
	#define SM_BITSHIFT_IIN_LO_TH			20

	#define SM_BITMASK_IIN_HI_TH			0b1111111111
	#define SM_BITSHIFT_IIN_HI_TH			30


	// CIE1931 correction table
	// Automatically generated

	//#############################################################################
	// Type definitions
	//-----------------------------------------------------------------------------
	typedef enum class pinPwm_timerType
	{
		timerTypeTCC,
		timerTypeTC
	} SM_pinPwm_timerType_t;
	

	//#############################################################################
	// Function prototypes
	//-----------------------------------------------------------------------------
	void VSYNC_IRQ(void);
	

	//#############################################################################
	// Shared variables
	//-----------------------------------------------------------------------------	
	extern uint32_t g_SM_pinId_pwmPin;
	

	//#############################################################################
	// Library 
	//-----------------------------------------------------------------------------
		
	// ----------------------------------------------------------------------------------		
	typedef enum class brtModeConfig
	{
		brtModePwmInputDutyCycleControl,
		brtModeBrightnessRegister,
		brtModeDirectControl,
		defaultValue
	} SM_brtModeConfig_t;	

	
	// ----------------------------------------------------------------------------------
	// Hardware Abstraction Layer

	void 	 SM72442_readRegister (uint8_t reg, char * regVals);
	uint64_t SM72442_readRegister64(uint8_t reg);
	
	void 	 SM72442_writeRegister(uint8_t reg, char * regVals);
	void 	 SM72442_writeRegister64(uint8_t reg, uint64_t regvalue);

	void 	 SM72442_set64(uint64_t value, char buffer[8]);
	uint64_t SM72442_get64(char buffer[8]);

	// --------------------------------------------------------------------------
	// Reg	0

	void SM72442_readADC(
		int32_t * adc6, 
		int32_t * adc4, 
		int32_t * adc2, 
		int32_t * adc0
	);

	// --------------------------------------------------------------------------
	// Reg 1

	void SM72442_readIV(
		int32_t * Vout, 
		int32_t * Iout, 
		int32_t * Vin, 
		int32_t * Iin
	);

	// --------------------------------------------------------------------------
	// Register 3
	// Known State

	extern bool        SM72242_STATE_override_adcprog;
	extern bool        SM72242_STATE_power_thresh_select;
	extern uint8_t     SM72242_STATE_bb_in_ptmode_sel;
	extern int32_t     SM72242_STATE_iout_max;
	extern int32_t     SM72242_STATE_vout_max;
	extern uint8_t     SM72242_STATE_deadtime_off;
	extern uint8_t     SM72242_STATE_deadtime_on;
	extern int32_t     SM72242_STATE_duty_cycle_open;
	extern bool        SM72242_STATE_pass_thru_select;
	extern bool        SM72242_STATE_pass_thru_manual;
	extern bool        SM72242_STATE_soft_reset;
	extern bool        SM72242_STATE_pll_clock;
	extern bool        SM72242_STATE_open_loop_enable;

	void SM72442_readConfig(
		bool * override_adcprog, 
		bool * power_thresh_select,
		uint8_t * bb_in_ptmode_sel,
		int32_t * iout_max,
		int32_t * vout_max,
		uint8_t * deadtime_off,
		uint8_t * deadtime_on,
		int32_t * duty_cycle_open,
		bool * pass_thru_select, 
		bool * pass_thru_manual,
		bool * soft_reset,
		bool * pll_clock,
		bool * open_loop_enable
	);

	void SM72442_STATE_SYNC_config(void);

	void SM72442_setConfig_override_adcprog(bool override_adcprog);
	void SM72442_setConfig_iout_max(int32_t iout_max);
	void SM72442_setConfig_vout_max(int32_t vout_max);
	void SM72442_setConfig_deadtime_off(uint8_t deadtime_off);
	void SM72442_setConfig_deadtime_on(uint8_t deadtime_on);
	void SM72442_setConfig_override_soft_reset(bool soft_reset);

	// --------------------------------------------------------------------------
	// Reg 4

	void SM72442_readOffsets(
		uint8_t * Vout_offset, 
		uint8_t * Iout_offset, 
		uint8_t * Vin_offset, 
		uint8_t * Iin_offset
	);

	void SM72442_setConfig_Vout_offset(int32_t Vout_offset);
	void SM72442_setConfig_Vout_offset(int32_t Iout_offset);
	void SM72442_setConfig_Vin_offset(int32_t Vin_offset);
	void SM72442_setConfig_Iin_offset(int32_t Iin_offset);

	// --------------------------------------------------------------------------
	// Reg 5

	void SM72442_readCurrentThresholds(
		int32_t * iout_lo_th, 
		int32_t * iout_hi_th, 
		int32_t * iin_lo_th, 
		int32_t * iin_hi_th
	);

	void SM72442_setConfig_iout_lo_th(int32_t iout_lo_th);
	void SM72442_setConfig_iout_hi_th(int32_t iout_hi_th);
	void SM72442_setConfig_iin_lo_th(int32_t iin_lo_th);
	void SM72442_setConfig_iin_hi_th(int32_t iin_hi_th);


#endif
