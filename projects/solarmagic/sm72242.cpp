/**
* @file lp8545.cpp
*
* @brief SM72242 Driver
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

// ##################################################################################
// Includes
// ----------------------------------------------------------------------------------
#include "sm72242.h"

#include <wiring_private.h>


// ##################################################################################
// Macros used in this CPP file
// ----------------------------------------------------------------------------------
#ifndef lowByte
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#endif

#ifndef highByte
#define highByte(w) ((uint8_t) ((w) >> 8))
#endif

#ifndef countBytes
#define countBytes(a) (sizeof(a)/sizeof(a[0]))
#endif


// ##################################################################################
// Global variables
// ----------------------------------------------------------------------------------
uint8_t g_sm_i2c_address = SM_DEFAULT_I2C_ADDRESS;

extern TwoWire I2C_BUS;

// ##################################################################################
// Function definitions
// ----------------------------------------------------------------------------------


// ##################################################################################
// SM72242_ Class
// ----------------------------------------------------------------------------------
// Sensor object

// ----------------------------------------------------------------------------------
// Private
// ----------------------------------------------------------------------------------
// Hardware abstraction layer

void SM72442_readRegister(
	uint8_t reg, 
	char * regVals
) {
	// Wake up device
	I2C_BUS.beginTransmission(SM_DEFAULT_I2C_ADDRESS);

	// Tell dev register we want to access
	I2C_BUS.write(reg);

	// Repeated start
	I2C_BUS.endTransmission(false);

	// Get Data Frames
	I2C_BUS.requestFrom(SM_DEFAULT_I2C_ADDRESS, SM_REGISTER_LENGTH_BYTES);

	uint8_t byte = 0;
	while (I2C_BUS.available()) {
		if(byte > SM_REGISTER_LENGTH_BYTES) {
			break;
		}
		regVals[byte] = I2C_BUS.read();
		byte++;
	}

	// Terminate
	I2C_BUS.endTransmission(true);
}


uint64_t SM72442_readRegister64(uint8_t reg) {

	// Buffer for values
	char regVals[8] = {0};
	
	// Get values
	SM72442_readRegister(reg, regVals);

	// Get whole register as integer
	uint64_t integer = SM72442_get64(regVals);

	// Return 64 bit integer
	return integer;
}


void SM72442_writeRegister(
	uint8_t reg, 
	char * regVals
) {
	// Wake up device
	I2C_BUS.beginTransmission(SM_DEFAULT_I2C_ADDRESS);

	// Tell dev register we want to access
	I2C_BUS.write(reg);

	// Length byte = 7
	I2C_BUS.write(7);

	uint8_t byte = 1;
	while (byte < SM_REGISTER_LENGTH_BYTES) {
		I2C_BUS.write(regVals[byte]);
		byte++;
	}

	// Terminate
	I2C_BUS.endTransmission(true);
}


void SM72442_writeRegister64(uint8_t reg, uint64_t regvalue) {
	
	// Buffer for values
	char regVals[8] = {0};

	// Convert uint64_t to buffer[]
	SM72442_set64(regvalue, regVals);
	
	// Set values
	SM72442_writeRegister(reg, regVals);
}


uint64_t SM72442_get64(char buffer[8])
{
	uint64_t integer = 

	(uint64_t)buffer[7] << 48 |
	(uint64_t)buffer[6] << 40 |
	(uint64_t)buffer[5] << 32 |
	(uint64_t)buffer[4] << 24 |
	(uint64_t)buffer[3] << 16 |
	(uint64_t)buffer[2] << 8  |
	(uint64_t)buffer[1];

	return integer;
}


void SM72442_set64(
	uint64_t value, 
	char buffer[8]
) {
	buffer[7] = (uint8_t)((uint64_t)(value >> 48) & 0xFF);
	buffer[6] = (uint8_t)((uint64_t)(value >> 40) & 0xFF);
	buffer[5] = (uint8_t)((uint64_t)(value >> 32) & 0xFF);
	buffer[4] = (uint8_t)((uint64_t)(value >> 24) & 0xFF);
	buffer[3] = (uint8_t)((uint64_t)(value >> 16) & 0xFF);
	buffer[2] = (uint8_t)((uint64_t)(value >> 8 ) & 0xFF);
	buffer[1] = (uint8_t)((uint64_t)(value >> 0 ) & 0xFF);
}


// Register 0
void SM72442_readADC(
	int32_t * adc6, 
	int32_t * adc4, 
	int32_t * adc2, 
	int32_t * adc0
) {	
	// Buffer for values
	char regVals[8] = {0};
	
	// Get values
	SM72442_readRegister(SM_REGISTER_REG_0, regVals);

	// Get whole register as integer
	uint64_t adc_integer = SM72442_get64(regVals);

	// Get ADC values
	*adc0 = (int32_t)((uint64_t)(adc_integer >> SM_BITSHIFT_ADC0 & SM_BITMASK_ADC0));
	*adc2 = (int32_t)((uint64_t)(adc_integer >> SM_BITSHIFT_ADC2 & SM_BITMASK_ADC2));
	*adc4 = (int32_t)((uint64_t)(adc_integer >> SM_BITSHIFT_ADC4 & SM_BITMASK_ADC4));
	*adc6 = (int32_t)((uint64_t)(adc_integer >> SM_BITSHIFT_ADC6 & SM_BITMASK_ADC6));
}


// Register 1
void SM72442_readIV(
	int32_t * Vout, 
	int32_t * Iout, 
	int32_t * Vin, 
	int32_t * Iin
) {
	// Get whole register as integer
	uint64_t integer = SM72442_readRegister64(SM_REGISTER_REG_1);

	// Get IV
	*Vout = (int32_t)((uint64_t)(integer >> SM_BITSHIFT_VOUT & SM_BITMASK_VOUT));
	*Iout = (int32_t)((uint64_t)(integer >> SM_BITSHIFT_IIN  & SM_BITMASK_IIN));
	*Vin  = (int32_t)((uint64_t)(integer >> SM_BITSHIFT_VIN  & SM_BITMASK_VIN));
	*Iin  = (int32_t)((uint64_t)(integer >> SM_BITSHIFT_IOUT & SM_BITMASK_IOUT));
}


// --------------------------------------------------------------------------
// Register 3 

bool        SM72242_STATE_override_adcprog;
bool        SM72242_STATE_power_thresh_select;
uint8_t     SM72242_STATE_bb_in_ptmode_sel;
int32_t     SM72242_STATE_iout_max;
int32_t     SM72242_STATE_vout_max;
uint8_t     SM72242_STATE_deadtime_off;
uint8_t     SM72242_STATE_deadtime_on;
int32_t     SM72242_STATE_duty_cycle_open;
bool        SM72242_STATE_pass_thru_select;
bool        SM72242_STATE_pass_thru_manual;
bool        SM72242_STATE_soft_reset;
bool        SM72242_STATE_pll_clock;
bool        SM72242_STATE_open_loop_enable;

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
) {
	// Get whole register as integer
	uint64_t integer = SM72442_readRegister64(SM_REGISTER_REG_3);

	// Get Config
	*override_adcprog 		= (bool)((uint64_t)(integer >> SM_BITSHIFT_OVERRIDE_ADCPROG 	& SM_BITMASK_OVERRIDE_ADCPROG));
	*power_thresh_select 	= (bool)((uint64_t)(integer >> SM_BITSHIFT_POWER_THR_SEL  		& SM_BITMASK_POWER_THR_SEL));
	*bb_in_ptmode_sel  		= (uint8_t)((uint64_t)(integer >> SM_BITSHIFT_BB_IN_PT_MODE_SEL & SM_BITMASK_BB_IN_PT_MODE_SEL));
	*iout_max  				= (int32_t)((uint64_t)(integer >> SM_BITSHIFT_IOUT_MAX 			& SM_BITMASK_IOUT_MAX));
	*vout_max  				= (int32_t)((uint64_t)(integer >> SM_BITSHIFT_VOUT_MAX 			& SM_BITMASK_VOUT_MAX));
	*deadtime_off  			= (uint8_t)((uint64_t)(integer >> SM_BITSHIFT_DEADTIME_OFF 		& SM_BITMASK_DEADTIME_OFF));
	*deadtime_on  			= (uint8_t)((uint64_t)(integer >> SM_BITSHIFT_DEADTIME_ON 		& SM_BITMASK_DEADTIME_ON));
	*duty_cycle_open  		= (int32_t)((uint64_t)(integer >> SM_BITSHIFT_DUTY_CYCLE_OPEN 	& SM_BITMASK_DUTY_CYCLE_OPEN));
	*pass_thru_select  		= (bool)((uint64_t)(integer >> SM_BITSHIFT_PASS_THRU_SEL 		& SM_BITMASK_PASS_THRU_SEL));
	*pass_thru_manual  		= (bool)((uint64_t)(integer >> SM_BITSHIFT_PASS_THRU_MAN 		& SM_BITMASK_PASS_THRU_MAN));
	*soft_reset		  		= (bool)((uint64_t)(integer >> SM_BITSHIFT_RESET 				& SM_BITMASK_RESET));
	*pll_clock		  		= (bool)((uint64_t)(integer >> SM_BITSHIFT_PLL_CLOCK			& SM_BITMASK_PLL_CLOCK));
	*open_loop_enable  		= (bool)((uint64_t)(integer >> SM_BITSHIFT_OPEN_LOOP			& SM_BITMASK_OPEN_LOOP));
}

void SM72442_STATE_SYNC_config(void) {
	SM72442_readConfig(
		&SM72242_STATE_override_adcprog, 
		&SM72242_STATE_power_thresh_select,
		&SM72242_STATE_bb_in_ptmode_sel,
		&SM72242_STATE_iout_max,
		&SM72242_STATE_vout_max,
		&SM72242_STATE_deadtime_off,
		&SM72242_STATE_deadtime_on,
		&SM72242_STATE_duty_cycle_open,
		&SM72242_STATE_pass_thru_select, 
		&SM72242_STATE_pass_thru_manual,
		&SM72242_STATE_soft_reset,
		&SM72242_STATE_pll_clock,
		&SM72242_STATE_open_loop_enable
	);
}

void SM72442_setConfig_override_adcprog(bool override_adcprog)
{
	uint64_t bitMaskValue = ~((uint64_t)SM_BITMASK_OVERRIDE_ADCPROG << SM_BITSHIFT_OVERRIDE_ADCPROG);
	uint64_t regValue = SM72442_readRegister64(SM_REGISTER_REG_3);
	uint64_t blankValue = regValue & bitMaskValue;
	uint64_t newValue = blankValue | (((uint64_t) override_adcprog & SM_BITMASK_OVERRIDE_ADCPROG) << SM_BITSHIFT_OVERRIDE_ADCPROG);
	SM72442_writeRegister64(SM_REGISTER_REG_3, newValue);
}

void SM72442_setConfig_iout_max(int32_t iout_max) 
{
	uint64_t bitMaskValue = ~((uint64_t)SM_BITMASK_IOUT_MAX << SM_BITSHIFT_IOUT_MAX);
	uint64_t regValue = SM72442_readRegister64(SM_REGISTER_REG_3);
	uint64_t blankValue = regValue & bitMaskValue;
	uint64_t newValue = blankValue | (((uint64_t) iout_max & SM_BITMASK_IOUT_MAX) << SM_BITSHIFT_IOUT_MAX);
	SM72442_writeRegister64(SM_REGISTER_REG_3, newValue);
}

void SM72442_setConfig_vout_max(int32_t vout_max) 
{
	uint64_t bitMaskValue = ~((uint64_t)SM_BITMASK_VOUT_MAX << SM_BITSHIFT_VOUT_MAX);
	uint64_t regValue = SM72442_readRegister64(SM_REGISTER_REG_3);
	uint64_t blankValue = regValue & bitMaskValue;
	uint64_t newValue = blankValue | (((uint64_t) vout_max & SM_BITMASK_VOUT_MAX) << SM_BITSHIFT_VOUT_MAX);
	SM72442_writeRegister64(SM_REGISTER_REG_3, newValue);
}

void SM72442_setConfig_deadtime_off(uint8_t deadtime_off)
{
	uint64_t bitMaskValue = ~((uint64_t)SM_BITMASK_DEADTIME_OFF << SM_BITSHIFT_DEADTIME_OFF);
	uint64_t regValue = SM72442_readRegister64(SM_REGISTER_REG_3);
	uint64_t blankValue = regValue & bitMaskValue;
	uint64_t newValue = blankValue | (((uint64_t) deadtime_off & SM_BITMASK_DEADTIME_OFF) << SM_BITSHIFT_DEADTIME_OFF);
	SM72442_writeRegister64(SM_REGISTER_REG_3, newValue);
}

void SM72442_setConfig_deadtime_on(uint8_t deadtime_on)
{
	uint64_t bitMaskValue = ~((uint64_t)SM_BITMASK_DEADTIME_ON << SM_BITSHIFT_DEADTIME_ON);
	uint64_t regValue = SM72442_readRegister64(SM_REGISTER_REG_3);
	uint64_t blankValue = regValue & bitMaskValue;
	uint64_t newValue = blankValue | (((uint64_t) deadtime_on & SM_BITMASK_DEADTIME_ON) << SM_BITSHIFT_DEADTIME_ON);
	SM72442_writeRegister64(SM_REGISTER_REG_3, newValue);
}

void SM72442_setConfig_override_soft_reset(bool soft_reset)
{
	uint64_t bitMaskValue = ~((uint64_t)SM_BITMASK_RESET << SM_BITSHIFT_RESET);
	uint64_t regValue = SM72442_readRegister64(SM_REGISTER_REG_3);
	uint64_t blankValue = regValue & bitMaskValue;
	uint64_t newValue = blankValue | (((uint64_t) soft_reset & SM_BITMASK_RESET) << SM_BITSHIFT_RESET);
	SM72442_writeRegister64(SM_REGISTER_REG_3, newValue);
}

// --------------------------------------------------------------------------
// Register 4

void SM72442_readOffsets(
	uint8_t * Vout_offset, 
	uint8_t * Iout_offset, 
	uint8_t * Vin_offset, 
	uint8_t * Iin_offset
) {
	// Get whole register as integer
	uint64_t integer = SM72442_readRegister64(SM_REGISTER_REG_4);

	// Get ADC values
	*Vout_offset = (uint8_t)((uint64_t)(integer >> SM_BITSHIFT_VOUT_OFFSET & SM_BITMASK_VOUT_OFFSET));
	*Iout_offset = (uint8_t)((uint64_t)(integer >> SM_BITSHIFT_IIN_OFFSET  & SM_BITMASK_IIN_OFFSET));
	*Vin_offset  = (uint8_t)((uint64_t)(integer >> SM_BITSHIFT_VIN_OFFSET  & SM_BITMASK_VIN_OFFSET));
	*Iin_offset  = (uint8_t)((uint64_t)(integer >> SM_BITSHIFT_IOUT_OFFSET & SM_BITMASK_IOUT_OFFSET));
}

void SM72442_setConfig_Vout_offset(int32_t Vout_offset) 
{
	uint64_t bitMaskValue = ~((uint64_t)SM_BITMASK_VOUT_OFFSET << SM_BITSHIFT_VOUT_OFFSET);
	uint64_t regValue = SM72442_readRegister64(SM_REGISTER_REG_4);
	uint64_t blankValue = regValue & bitMaskValue;
	uint64_t newValue = blankValue | (((uint64_t) Vout_offset & SM_BITMASK_VOUT_OFFSET) << SM_BITSHIFT_VOUT_OFFSET);
	SM72442_writeRegister64(SM_REGISTER_REG_4, newValue);
}

void SM72442_setConfig_Iout_offset(int32_t Iout_offset) 
{
	uint64_t bitMaskValue = ~((uint64_t)SM_BITMASK_IOUT_OFFSET << SM_BITSHIFT_IOUT_OFFSET);
	uint64_t regValue = SM72442_readRegister64(SM_REGISTER_REG_4);
	uint64_t blankValue = regValue & bitMaskValue;
	uint64_t newValue = blankValue | (((uint64_t) Iout_offset & SM_BITMASK_IOUT_OFFSET) << SM_BITSHIFT_IOUT_OFFSET);
	SM72442_writeRegister64(SM_REGISTER_REG_4, newValue);
}

void SM72442_setConfig_Vin_offset(int32_t Vin_offset) 
{
	uint64_t bitMaskValue = ~((uint64_t)SM_BITSHIFT_VIN_OFFSET << SM_BITSHIFT_VIN_OFFSET);
	uint64_t regValue = SM72442_readRegister64(SM_REGISTER_REG_4);
	uint64_t blankValue = regValue & bitMaskValue;
	uint64_t newValue = blankValue | (((uint64_t) Vin_offset & SM_BITSHIFT_VIN_OFFSET) << SM_BITSHIFT_VIN_OFFSET);
	SM72442_writeRegister64(SM_REGISTER_REG_4, newValue);
}

void SM72442_setConfig_Iin_offset(int32_t Iin_offset) 
{
	uint64_t bitMaskValue = ~((uint64_t)SM_BITMASK_IIN_OFFSET << SM_BITSHIFT_IIN_OFFSET);
	uint64_t regValue = SM72442_readRegister64(SM_REGISTER_REG_4);
	uint64_t blankValue = regValue & bitMaskValue;
	uint64_t newValue = blankValue | (((uint64_t) Iin_offset & SM_BITMASK_IIN_OFFSET) << SM_BITSHIFT_IIN_OFFSET);
	SM72442_writeRegister64(SM_REGISTER_REG_4, newValue);
}


// --------------------------------------------------------------------------
// Register 5

void SM72442_readCurrentThresholds(
	int32_t * iin_hi_th, 
	int32_t * iin_lo_th, 
	int32_t * iout_hi_th, 
	int32_t * iout_lo_th
) {
	// Get whole register as integer
	uint64_t integer = SM72442_readRegister64(SM_REGISTER_REG_5);

	// Get ADC values
	*iout_lo_th 	= (int32_t)((uint64_t)(integer >> SM_BITSHIFT_IOUT_LO_TH 	& SM_BITMASK_IOUT_LO_TH));
	*iout_hi_th 	= (int32_t)((uint64_t)(integer >> SM_BITSHIFT_IOUT_HI_TH 	& SM_BITMASK_IOUT_HI_TH));
	*iin_lo_th 		= (int32_t)((uint64_t)(integer >> SM_BITSHIFT_IIN_LO_TH 	& SM_BITMASK_IIN_LO_TH));
	*iin_hi_th 		= (int32_t)((uint64_t)(integer >> SM_BITSHIFT_IIN_HI_TH 	& SM_BITMASK_IIN_HI_TH));
}

void SM72442_setConfig_iout_lo_th(int32_t iout_lo_th)
{
	uint64_t bitMaskValue = ~((uint64_t)SM_BITMASK_IOUT_LO_TH << SM_BITSHIFT_IOUT_LO_TH);
	uint64_t regValue = SM72442_readRegister64(SM_REGISTER_REG_5);
	uint64_t blankValue = regValue & bitMaskValue;
	uint64_t newValue = blankValue | (((uint64_t) iout_lo_th & SM_BITMASK_IOUT_LO_TH) << SM_BITSHIFT_IOUT_LO_TH);
	SM72442_writeRegister64(SM_REGISTER_REG_5, newValue);
}

void SM72442_setConfig_iout_hi_th(int32_t iout_hi_th)
{
	uint64_t bitMaskValue = ~((uint64_t)SM_BITMASK_IOUT_HI_TH << SM_BITSHIFT_IOUT_HI_TH);
	uint64_t regValue = SM72442_readRegister64(SM_REGISTER_REG_5);
	uint64_t blankValue = regValue & bitMaskValue;
	uint64_t newValue = blankValue | (((uint64_t) iout_hi_th & SM_BITMASK_IOUT_HI_TH) << SM_BITSHIFT_IOUT_HI_TH);
	SM72442_writeRegister64(SM_REGISTER_REG_5, newValue);
}

void SM72442_setConfig_iin_lo_th(int32_t iin_lo_th)
{
	uint64_t bitMaskValue = ~((uint64_t)SM_BITMASK_IIN_LO_TH << SM_BITSHIFT_IIN_LO_TH);
	uint64_t regValue = SM72442_readRegister64(SM_REGISTER_REG_5);
	uint64_t blankValue = regValue & bitMaskValue;
	uint64_t newValue = blankValue | (((uint64_t) iin_lo_th & SM_BITMASK_IIN_LO_TH) << SM_BITSHIFT_IIN_LO_TH);
	SM72442_writeRegister64(SM_REGISTER_REG_5, newValue);
}

void SM72442_setConfig_iin_hi_th(int32_t iin_hi_th)
{
	uint64_t bitMaskValue = ~((uint64_t)SM_BITMASK_IIN_HI_TH << SM_BITSHIFT_IIN_HI_TH);
	uint64_t regValue = SM72442_readRegister64(SM_REGISTER_REG_5);
	uint64_t blankValue = regValue & bitMaskValue;
	uint64_t newValue = blankValue | (((uint64_t) iin_hi_th & SM_BITMASK_IIN_HI_TH) << SM_BITSHIFT_IIN_HI_TH);
	SM72442_writeRegister64(SM_REGISTER_REG_5, newValue);
}