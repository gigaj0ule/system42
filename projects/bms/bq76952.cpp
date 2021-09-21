/**
* @file bq76952.cpp
*
* @brief BQ76952 Driver
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
#include "bq76952.h"

#include <Arduino.h>

//#include "../pin_settings.h"
#include <stdbool.h>
#include <Wire.h>

TwoWire myWire (PB11, PB10);


// ##################################################################################
// Macros used in this CPP file
// ----------------------------------------------------------------------------------
#ifndef lowByte
#define lowByte(w) ((uint8_t) ((w) & 0xff))
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef highByte
#define highByte(w) ((uint8_t) ((w) >> 8))
#endif

#ifndef countBytes
#define countBytes(a) (sizeof(a)/sizeof(a[0]))
#endif

#define EASING_STEPS 50
#define EASING_IRQ_FREQUENCY 50

// ##################################################################################
// Global variables
// ----------------------------------------------------------------------------------

float   easingFunctionLut[EASING_STEPS];
bool g_LP_vsync_timer_isRunning = false;
bool g_LP_easing_timer_isRunning = false;
bool g_LP_pinPwm_oneShotModeEnabled;


// <bugfix on STM32>
// @todo: understand why
#include <errno.h>
int *__errno (void) {
  static int foo;
  return &foo;
}
// </bugfix>


// Pins
int g_LP_pwmPin;

// Brightness state
typedef struct
{
	int16_t startingValue;
	int16_t targetValue;
	int16_t currentValue;
	int16_t lastBrightnessSetting;
	
	int16_t easingValue;
	int8_t easingDirection;
	int8_t easingLutIndex;
	
	bool haltSemaphore;
	bool boostEnabled;
	bool advancedSlopeEnabled;
} brightnessState_t;

brightnessState_t g_LP_brightnessState;



// ##################################################################################
// Function definitions
// ----------------------------------------------------------------------------------
#include <cmsis_os.h>

// @brief Halts interrupts for critical task completion
// @note These functions enable and disable interrupts (just like their cousins in low_level.h) 
// but must be a static part of this file to be truly inlined
// @retval priority_mask - cached interrupt mask
static inline uint32_t fast__cpu_enter_critical() {
    uint32_t priority_mask = __get_PRIMASK();
    __disable_irq();
    return priority_mask;
}

// @brief Resumes interrupts for critical task completion
// @note These functions enable and disable interrupts (just like their cousins in low_level.h) 
// but must be a static part of this file to be truly inlined
// @param priority_mask - Interrupt mask to restore
static inline void fast__cpu_exit_critical(uint32_t priority_mask) {
    __set_PRIMASK(priority_mask);
}

// ##################################################################################
// BQ76952_ Class
// ----------------------------------------------------------------------------------
// Sensor object
BQ76952_ g_BQ76952;

BQ76952_::BQ76952_(void)
{
}

void BQ76952_ :: begin(bool verifyRegisters)
{	
	// Debug (verify chip writes?)
	this->LP_debugVerifyRegisters = verifyRegisters;

	// Init I2C
	//myWire.begin();
}

bool BQ76952_ :: setup()
{
	uint16_t status = 0;
	getCtlStatus(&status);

	__asm__ volatile ("nop");

	status = 0;
}

void BQ76952_ :: loopTask(void)
{
}

void BQ76952_ :: end(void)
{
}

void BQ76952_ :: brightness(uint16_t pwmVal, bool shouldFade)
{
}

void BQ76952_ :: powerOn(void)
{
}

void BQ76952_ :: powerOff(void)
{
}

void BQ76952_ :: boostOn(void)
{
}

void BQ76952_ :: boostOff(void)
{
}

void BQ76952_ :: lightOn(bool shouldFade)
{
}

void BQ76952_ :: lightOff(bool shouldFade)
{
}



// ----------------------------------------------------------------------------------
// Configuration
void BQ76952_ :: setFactoryDefaults(void)
{
	// Eeprom 1
	/*
	setBoostFreq(boostFreqConfig::defaultValue);
	setLedFaultDetection(ledFaultsConfig::defaultValue);
	setTempLimit(tempLimitConfig::defaultValue);
	setSlope(slopeConfig::defaultValue, advancedSlopeConfig::defaultValue);
	
	// Eeprom 2
	setAdaptiveSpeed(adaptiveSpeedConfig1::defaultValue, adaptiveSpeedConfig2::defaultValue);
	setFetEnable(enExternalFetConfig::defaultValue);
	setAdaptiveMode(enAdaptiveModeConfig::defaultValue);
	setBoostOnOff(enableBoostConfig::defaultValue);
	setBoostCurrent(boostMaxCurrentConfig::defaultValue);

	// Eeprom 3
	setUVLO(LP_uvloConfig_t::defaultValue);
	setPhaseShiftPWM(LP_enPhaseShiftPwmConfig_t::defaultValue);
	setPWMFreq(0);
	
	// Eeprom 4
	setPwmResolution(LP_pwmResolutionConfig_t::defaultValue);
	setIresEnable(LP_enIresConfig_t::defaultValue);
	setLedFaultThreshold (LP_ledFaultThresholdConfig_t::defaultValue);
	setLedHeadrooom (LP_ledHeadroomConfig_t::defaultValue);
	
	// Eeprom 5
	setVsync(LP_enVsyncConfig_t::defaultValue);
	setDither(LP_ditherConfig_t::defaultValue);
	setVboost(0);
	
	// Eeprom 6
	setPll(3205);
	setFresEnabled(LP_enFresConfig_t::defaultValue);
	setHysteresisResolution(LP_hysteresisConfig_t::defaultValue);
	*/
}

// ----------------------------------------------------------------------------------
// LP_REGISTER_CTL_STATUS
bool BQ76952_ :: getCtlStatus(uint16_t * status_reg)
{
	uint8_t pData[4] = {0};
	readRegister(LP_REGISTER_CTL_STATUS, pData, 2);
	
	//__asm__ volatile ("nop");

	//uint8_t newRegister = (oldRegister & ~LP_BITMASK_BRT) | (LP_BITMASK_BRT & setting_bits);
	//setRegister(LP_REGISTER_CTL_STATUS, newRegister);

	/*if(this->LP_debugVerifyRegisters == true)
	{
		//uint8_t verifyRegister;
		//readRegister(LP_REGISTER_BRIGHT_CTL, &verifyRegister);
		//return (newRegister == verifyRegister ? false : true);
		return true;
	}
	else
	{
		return true;
	}*/
}

// ----------------------------------------------------------------------------------
// LP_REGISTER_DEVICE_CTL
bool BQ76952_ :: setBrtMode(LP_brtModeConfig_t enumerator)
{
	uint8_t setting_bits;
	
	switch(enumerator){
		case brtModeConfig::brtModePwmInputDutyCycleControl:
		case brtModeConfig::defaultValue:
		{
			setting_bits = 0b00000000;
			break;
		}
		case brtModeConfig::brtModeBrightnessRegister:
		default:
		{
			setting_bits = 0b00000100;
			break;
		}
		case brtModeConfig::brtModeDirectControl:
		{
			setting_bits = 0b00000110;
			break;
		}
	}
	
	/*
	uint8_t oldRegister;
	readRegister(LP_REGISTER_DEVICE_CTL, &oldRegister);
	
	uint8_t newRegister = (oldRegister & ~LP_BITMASK_BRT_MODE) | (LP_BITMASK_BRT_MODE & setting_bits);
	setRegister(LP_REGISTER_DEVICE_CTL, newRegister);
	
	if(this->LP_debugVerifyRegisters == true)
	{
		uint8_t verifyRegister;
		readRegister(LP_REGISTER_DEVICE_CTL, &verifyRegister);
		return (newRegister == verifyRegister ? false : true);
	}
	else
	{
		return true;
	}
	*/
}


// ----------------------------------------------------------------------------------
// Misc
uint32_t BQ76952_ :: mapResolution(uint32_t value, uint32_t from, uint32_t to)
{
	if (from == to)
	{
		return value;
	}
	if (from > to)
	{
		return value >> (from-to);
	}
	return value << (to-from);
}

BQ76952_ BQ76952;






// ----------------------------------------------------------------------------------
// Private
// ----------------------------------------------------------------------------------
// Hardware abstraction layer
void setRegister(uint8_t reg, uint8_t dataByte)
{
	uint8_t pData[2];
	
	pData[0] = reg;
	pData[1] = dataByte;
	
	i2c_tx(LP_I2C_ADDRESS, &pData[0], countBytes(pData));
}


void readRegister(uint8_t register_address, uint8_t * register_data, uint8_t length)
{
	// Room for optimization here...
	uint8_t pData[1];
	pData[0] = register_address;
	
	// hmmm
	//uint32_t mask = fast__cpu_enter_critical();

	// Transmit address	
	i2c_tx(LP_I2C_ADDRESS, pData, 1);
	
	// Request N bytes from device
	i2c_rx(LP_I2C_ADDRESS, register_data, length);
	//fast__cpu_exit_critical(mask);
}

void readRegisterDebug(uint8_t reg)
{
	uint8_t pData[1];
	uint8_t readByte;

	pData[0] = reg;
	
	i2c_tx(LP_I2C_ADDRESS, &pData[0], countBytes(pData));

	i2c_rx(LP_I2C_ADDRESS, &readByte, countBytes(pData));
	
	#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
	#define BYTE_TO_BINARY(byte)  \
	(byte & 0x80 ? '1' : '0'), \
	(byte & 0x40 ? '1' : '0'), \
	(byte & 0x20 ? '1' : '0'), \
	(byte & 0x10 ? '1' : '0'), \
	(byte & 0x08 ? '1' : '0'), \
	(byte & 0x04 ? '1' : '0'), \
	(byte & 0x02 ? '1' : '0'), \
	(byte & 0x01 ? '1' : '0')
	
	/*Serial.print("0x");
	Serial.print(reg, HEX);
	Serial.print("0x%02x: 0b");
	Serial.print(readByte, HEX);
	Serial.print(" ");
	Serial.print(readByte);*/
}

void i2c_tx(uint8_t addr, uint8_t* pData, uint8_t len)
{
	myWire.beginTransmission(addr);
	
	for(uint8_t i = 0; i < len; i++){
		uint8_t databyte = (uint8_t)(pData[i]);
		myWire.write(databyte);
	}
	
	myWire.endTransmission();
}

void i2c_rx(uint8_t addr, uint8_t *pData, uint8_t len)
{
	myWire.requestFrom(addr, len);
		
	uint8_t *readPtr = pData;

	/// @todo: Might write over memory if pData too small...
	while(myWire.available())
	{
		*readPtr = myWire.read();
		readPtr++;
		//ack = true;
	}
}