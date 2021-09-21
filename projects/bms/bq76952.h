/**
* @file bq76952.h
*
* @brief BQ76952 Driver _H
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
 
#ifndef _BQ76952_H_
	#define _BQ76952_H_

	#include <Arduino.h>
	#include <stdbool.h>

	//#############################################################################
	// Macros
	//-----------------------------------------------------------------------------
	// Device address
	#define LP_I2C_ADDRESS					0x08

	// ----------------------------------------------------------------------------------
	// Register IDs
	#define LP_REGISTER_CTL_STATUS			0x00
	#define LP_REGISTER_DEVICE_CTL			0x01
	#define LP_REGISTER_FAULT				0x02
	#define LP_REGISTER_ID					0x03
	#define LP_REGISTER_DIRECT_CTL			0x04
	#define LP_REGISTER_TEMP_MSB			0x05
	#define LP_REGISTER_TEMP_LSB			0x06	
	#define LP_REGISTER_EEPROM_CTL			0x72
	#define LP_REGISTER_EEPROM_0			0xA0
	#define LP_REGISTER_EEPROM_1			0xA1
	#define LP_REGISTER_EEPROM_2			0xA2
	#define LP_REGISTER_EEPROM_3			0xA3
	#define LP_REGISTER_EEPROM_4			0xA4
	#define LP_REGISTER_EEPROM_5			0xA5
	#define LP_REGISTER_EEPROM_6			0xA6
	#define LP_REGISTER_EEPROM_7			0xA7

	// ----------------------------------------------------------------------------------
	// Brightness CTL
	#define LP_BITMASK_BRT					0b11111111

	// Device CTL
	#define LP_BITMASK_BRT_MODE				0b00000110
	#define LP_BITMASK_BL_CTL				0b00000001

	// Fault
	#define LP_BITMASK_FAULT_OPEN			0b10000000
	#define LP_BITMASK_FAULT_SHORT			0b01000000
	#define LP_BITMASK_FAULT_2CHAN			0b00100000
	#define LP_BITMASK_FAULT_1CHAN			0b00010000
	#define LP_BITMASK_FAULT_BL_FAULT		0b00001000
	#define LP_BITMASK_FAULT_OVERCURENT		0b00000100
	#define LP_BITMASK_FAULT_TSD			0b00000010
	#define LP_BITMASK_FAULT_UVLO			0b00000001

	// ID		
	#define LP_BITMASK_PANEL				0b10000000
	#define LB_BITMASK_MFG					0b01111000
	#define LB_BITMASK_REV					0b00000111

	// Direct Control
	#define LP_BITMASK_OUT_6				0b00100000
	#define LP_BITMASK_OUT_5				0b00010000
	#define LP_BITMASK_OUT_4				0b00001000
	#define LP_BITMASK_OUT_3				0b00000100
	#define LP_BITMASK_OUT_2				0b00000010
	#define LP_BITMASK_OUT_1				0b00000001

	// Temp	MSB, LSB	
	#define LP_BITMASK_TEMP_HIBITS			0b11111111
	#define LP_BITMASK_TEMP_LOWBITS			0b11000000

	// Eeprom CTL		
	#define LP_BITMASK_EE_READY				0b10000000
	#define LP_BITMASK_EE_INIT				0b00000100
	#define LP_BITMASK_EE_PROG				0b00000010
	#define LP_BITMASK_EE_READ				0b00000001

	// Eeprom 0		
	#define LP_BITMASK_CURRENT				0b11111111
		
	// Eeprom 1		
	#define LP_BITMASK_BOOST_FREQ			0b11000000
	#define LP_BITMASK_EN_LED_FAULT			0b00100000
	#define LP_BITMASK_TEMP_LIM				0b00011000
	#define LP_BITMASK_SLOPE				0b00000111

	// Eeprom 2		
	#define LP_BITMASK_ADAPTIVE_SPEED		0b11000000
	#define LP_BITMASK_ADV_SLOPE			0b00100000
	#define LP_BITMASK_EN_EXT_FET			0b00010000
	#define LP_BITMASK_EN_ADAPT				0b00001000
	#define LP_BITMASK_EN_BOOST				0b00000100
	#define LP_BITMASK_BOOST_MAX			0b00000011
	
	// Eeprom 3		
	#define LP_BITMASK_UVLO					0b11000000
	#define LP_BITMASK_EN_PSPWM				0b00100000
	#define LP_BITMASK_PWM_FREQ				0b00011111

	// Eeprom 4		
	#define LP_BITMASK_PWM_RESOLUTION		0b11000000
	#define LP_BITMASK_EN_IRES				0b00100000
	#define LP_BITMASK_LED_FAULT_THR		0b00011000
	#define LP_BITMASK_DRV_HEADR			0b00000111

	// Eeprom 5
	#define LP_BITMASK_EN_VSYNC				0b10000000
	#define LP_BITMASK_DITHER				0b01100000
	#define LP_BITMASK_VBOOST				0b00011111

	// Eeprom 6, 7
	#define LP_BITMASK_PLL_HIGHBITS			0b11111111
	#define LP_BITMASK_PLL_LOWBITS			0b11111000
	#define LP_BITMASK_EN_F_RES				0b00000100
	#define LP_BITMASK_HYSTERESIS			0b00000011

	// CIE1931 correction table
	// Automatically generated

	//#############################################################################
	// Type definitions
	//-----------------------------------------------------------------------------
	typedef enum class pinPwm_timerType
	{
		timerTypeTCC,
		timerTypeTC
	} LP_pinPwm_timerType_t;
	

	//#############################################################################
	// Function prototypes
	//-----------------------------------------------------------------------------
	void VSYNC_IRQ_SAMD21(void);
	

	//#############################################################################
	// Shared variables
	//-----------------------------------------------------------------------------	
	extern uint32_t g_LP_pinId_pwmPin;
	

	//#############################################################################
	// Library class
	//-----------------------------------------------------------------------------
	class BQ76952_
	{

		// ----------------------------------------------------------------------------------
		public:
		typedef enum class vsyncModeSetup
		{
			vsyncModeOneshot,
			vsyncModeAstableTriggered,
			vsyncModeAstableAutostart
		} LP_config_vsyncType_t;
		
		typedef enum class easingFunctionSetup
		{
			linear,
			quadratic,
			cubic,
			quartic,
			quintic,
			sinusoid,
			circular,
			exponential,
			elastic
		} LP_easingFunctionType_t;
		
		BQ76952_(void);
		void begin(bool verifyRegisters);
		void end(void);
		bool setup();
		void loopTask(void);
		void brightness(uint16_t pwmVal, bool shouldFade);
		void powerOn(void);
		void powerOff(void);
		void boostOn(void);
		void boostOff(void);
		void lightOn(bool shouldFade);
		void lightOff(bool shouldFade);
		uint8_t readClearFaults(void);
		void setFactoryDefaults(void);

				
		// LP_REGISTER_CTL_STATUS
		bool getCtlStatus(uint16_t * status_reg);


		// ----------------------------------------------------------------------------------
		private:
		bool LP_debugVerifyRegisters = 0;
		
		typedef struct
		{
			int pin_vsync_irq;	
		} LP_pinIDs_t;
		
		LP_pinIDs_t pinIDs;
		
		typedef enum class brtModeConfig
		{
			brtModePwmInputDutyCycleControl,
			brtModeBrightnessRegister,
			brtModeDirectControl,
			defaultValue
		} LP_brtModeConfig_t;	
	
		typedef enum class blCtlConfig
		{
			blCtlOff,
			blCtlOn,
			defaultValue
		} LP_blCtlConfig_t;
			
		// Eeprom 1	
		typedef enum class boostFreqConfig
		{
			boostFreq115kHz,
			boostFreq312kHz,
			boostFreq625kHz,
			boostFreq1250kHz,
			defaultValue	
		} LP_boostFreqConfig_t;

		typedef enum class ledFaultsConfig
		{
			ledFaultsDisabled,
			ledFaultsEnabled,
			defaultValue
		} LP_ledFaultsConfig_t;

		typedef enum class tempLimitConfig
		{
			tempLimitDisabled,
			tempLimit110C,
			tempLimit120C,
			tempLimit130C,
			defaultValue
		} LP_tempLimitConfig_t;

		typedef enum class slopeConfig
		{
			slopeDisabled,
			slope50ms,
			slope75ms,
			slope100ms,
			slope150ms,
			slope200ms,
			slope300ms,
			slope500ms,
			defaultValue
		} LP_slopeConfig_t;

		// Eeprom 2
		typedef enum class adaptiveSpeedConfig1
		{
			adaptiveSpeedNormalMode,
			adaptiveSpeedLightLoads,
			defaultValue
		} LP_adaptiveSpeedConfig_1_t;
	
		typedef enum class adaptiveSpeedConfig2
		{
			adaptiveSpeedOncePerCycle,
			adaptiveSpeedOncePer16Cycle,
			defaultValue
		} LP_adaptiveSpeedConfig_2_t;

		typedef enum class advancedSlopeConfig
		{
			advancedSlopeDisabled,
			advancedSlopeEnabled,
			defaultValue
		} LP_advancedSlopeConfig_t;

		typedef enum class enExternalFetConfig
		{
			externalFetDisabled,
			externalFetEnabled,
			defaultValue
		} LP_enExternalFetConfig_t;

		typedef enum class enAdaptiveModeConfig
		{
			adaptiveModeDisabled,
			adaptiveModeEnabled,
			defaultValue
		} LP_enAdaptiveModeConfig_t;

		typedef enum class enableBoostConfig
		{
			boostDisabled,
			boostEnabled,
			defaultValue
		} LP_enableBoostConfig_t;

		typedef enum class boostMaxCurrentConfig
		{
			maxBoostCurrent900mA,
			maxBoostCurrent1400mA,
			maxBoostCurrent2000mA,
			maxBoostCurrent2500mA,
			defaultValue
		} LP_boostMaxCurrentConfig_t;
	
		// Eeprom 3
		typedef enum class uvloConfig
		{
			uvloDisabled,
			uvlo2v7,
			uvlo6v,
			uvlo9v,
			defaultValue
		} LP_uvloConfig_t;	

		typedef enum class enPhaseShiftPwmConfig
		{
			phaseShiftPWMDisable,
			phaseShiftPWMEnable,
			defaultValue
		} LP_enPhaseShiftPwmConfig_t;
	
		// Eeprom 4	
		typedef enum class pwmResolutionConfig
		{
			pwmResolutionLOW,
			pwmResolutionMEDIUM,
			pwmResolutionHIGH,
			pwmResolutionTURNED_PAST_11,
			defaultValue
		} LP_pwmResolutionConfig_t;

		typedef enum class enIresConfig
		{
			iresDisabled,
			iresEnabled,
			defaultValue
		} LP_enIresConfig_t;

		typedef enum class ledFaultThresholdConfig
		{
			ledFaultThreshold2v3,
			ledFaultThreshold3v3,
			ledFaultThreshold4v3,
			ledFaultThreshold5v3,
			defaultValue	
		} LP_ledFaultThresholdConfig_t;

		typedef enum class ledHeadroomConfig
		{
			ledHeadroom125mV,
			ledHeadroom250mV,
			ledHeadroom375mV,
			ledHeadroom500mV,
			ledHeadroom625mV,
			ledHeadroom750mV,
			ledHeadroom875mV,
			ledMAXHeadroom1000mV,
			defaultValue	
		} LP_ledHeadroomConfig_t;

		// Eeprom 5
		typedef enum class enVsyncConfig
		{
			vsyncDisabled,
			vsyncEnabled,
			defaultValue		
		} LP_enVsyncConfig_t;

		typedef enum class ditherConfig
		{
			ditherDisabled,
			dither1bit,
			dither2bit,
			dither3bit,
			defaultValue
		} LP_ditherConfig_t;

		// Eeprom 6, 7
		typedef enum class enFresConfig
		{
			fresDisabled,
			fresEnabled,
			defaultValue
		} LP_enFresConfig_t;

		typedef enum class hysteresisConfig
		{
			hysteresisDisabled,
			hysteresis11bit,
			hysteresis10bit,
			hysteresis8bit,
			defaultValue		
		} LP_hysteresisConfig_t;
		

		
		// Config		

		// LP_REGISTER_DEVICE_CTL
		bool setBrtMode(LP_brtModeConfig_t enumerator);
		
		// Misc
		uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to);
	};			
	

	// ----------------------------------------------------------------------------------
	// Hardware Abstraction Layer
	void setRegister(uint8_t reg, uint8_t dataByte);
	void readRegister(uint8_t reg, uint8_t * readByte, uint8_t len);
	void readRegisterDebug(uint8_t reg);
	void i2c_tx(uint8_t addr, uint8_t * pData, uint8_t len);
	void i2c_rx(uint8_t addr, uint8_t * pData, uint8_t len);

	extern BQ76952_ BQ76952;	
		
#endif
