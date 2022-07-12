#include "SparkFun_bq769x0.h"

#define PIN_BQ_TS1 PB5
#define PIN_BQ_RESET PB4
#define AFE_ALERT PC15
#define AFE_I2C_ADDR 0x18


// The bq769x0 without CRC has the 7-bit address 0x08. 
// The bq769x0 with CRC has the address 0x18.
// Please see the datasheet for more info
uint8_t bq_I2CAddress = AFE_I2C_ADDR; 

// Keeps track of when the Alert pin has been raised
volatile bool bq769x0_IRQ_Triggered = false; 

// These are two internal factory set values.
// We read them once at boot up and use them in many functions
float bq_gain = 0; 
int   bq_offset = 0; 

// Keeps track of overall pack fuel gauage
long bq_totalCoulombCount = 0; 

// Interrupt enabled, connected to bq pin ALERT
uint8_t bq_irqPin; 

// I2C Object
TwoWire bqWire;

// ---------------------------------------------------------------------
// This is irq handler for bq769x0 interrupts, has to return void and take no arguments
// always make code in interrupt handlers fast and short
//
void bq769x0IRQ() {

	bq769x0_IRQ_Triggered = true;
}

// ---------------------------------------------------------------------
// Listen 
// 
bool bq769x0_listen() {
    
	// For every IRQ event read the flags and update the coulomb counter and other major events
	if(bq769x0_IRQ_Triggered == true) {

		// Read the status register and update if needed
		byte sysStat = bq769x0_registerRead(bq796x0_SYS_STAT);

		SerialUSB.print("sysStat: 0x");
		SerialUSB.println(sysStat, HEX);

		// Double check that ADC is enabled
		byte sysVal = bq769x0_registerRead(bq796x0_SYS_CTRL1);

		if(sysVal & bq796x0_ADC_EN) {
		    SerialUSB.println("ADC Enabled");
		}

		// We need to write 1s into all the places we want a zero, but not overwrite the 1s we want left alone
		byte sysNew = 0;

		// Check for couloumb counter read
		if(sysStat & bq796x0_CC_READY) {

			SerialUSB.println("CC Ready");

			// Add this 250ms reading to the global fuel gauge
			bq_totalCoulombCount += (int) bq769x0_readCoulombCounter();

			// Clear this status bit by writing a one into this spot
			sysNew |= bq796x0_CC_READY;
		}

		// Internal fault
		if(sysStat & bq796x0_DEVICE_XREADY) {

			SerialUSB.println("Internal fault");

			// Clear this status bit by writing a one into this spot
			sysNew |= bq796x0_DEVICE_XREADY;
		}

		// Alert pin is being pulled high externally?
		if(sysStat & bq796x0_OVRD_ALERT) {

			SerialUSB.println("Override alert");

			// Clear this status bit by writing a one into this spot
			sysNew |= bq796x0_OVRD_ALERT;
		}

		if(sysStat & bq796x0_UV) {
			// Under voltage
			SerialUSB.println("Under voltage alert!");

			// Clear this status bit by writing a one into this spot
			sysNew |= bq796x0_UV;
		}

		// Over voltage
		if(sysStat & bq796x0_OV) {

			SerialUSB.println("Over voltage alert!");

			// Clear this status bit by writing a one into this spot
			sysNew |= bq796x0_OV;
		}

		// Short circuit detect
		if(sysStat & bq796x0_SCD) {

			SerialUSB.println("Short Circuit alert!");

			// Clear this status bit by writing a one into this spot
			//sysNew |= bq796x0_SCD;
		}

		// Over current detect
		if(sysStat & bq796x0_OCD) {

			SerialUSB.println("Over current alert!");

			// Clear this status bit by writing a one into this spot
			//sysNew |= bq796x0_OCD;
		}

		// Update the SYS_STAT with only the ones we want, only these bits will clear to zero
		bq769x0_registerWrite(bq796x0_SYS_STAT, sysNew);

		// Reset IRQ flag
		bq769x0_IRQ_Triggered = false;
	}

    return 0;
}

double dround(double val, int dp) {

}

// ---------------------------------------------------------------------
// Round X to Y places
//
float bq76940_roundf(float number, int precision) {
    
    int decimal_modifier = pow(10, precision);

    int value = (number * decimal_modifier);

    float value_f = (float)value / decimal_modifier;

    return value_f;
}

// ---------------------------------------------------------------------
// Initiates the first few I2C commands
// Returns true if we can verify communication
// Set CC_CFG to default 0x19
// Turn on the ADC
// Assume we are checking internal die temperatures (leave TEMP_SEL at zero)
// Configure the interrupts for Arduino Uno
// Read the Gain and Offset factory settings into global variables
//
bool bq769x0_initBQ(TwoWire &passedWire, byte bq_irqPin) {

    // Assign wire 
    if(&passedWire != NULL) {
        bqWire = passedWire;
    }
    else {
        // Must have a wire object!
        return(-0);
    }

    // Begin wire object
	bqWire.begin();

	// Reset the IC
	bq769x0_reset();

	// Wake the IC
	bq769x0_wake();

	// Test to see if we have correct I2C communication
	byte testByte = bq769x0_registerRead(bq796x0_SYS_STAT); 

    // Should be something other than zero on POR
	for(byte x = 0 ; x < 10 && testByte == 0 ; x++) {
		// SerialUSB.println(".");
		testByte = bq769x0_registerRead(bq796x0_SYS_STAT);
		delay(100);
	}

	if(testByte == 0x00) {
		// Something is very wrong. Check wiring.
		return false;
	}

	// "For optimal performance, [CC_CFG] should be programmed to 0x19 upon device startup." page 40
	bq769x0_registerWrite(bq796x0_CC_CFG, 0x19);

	// Double check that ADC is enabled
	byte sysVal = bq769x0_registerRead(bq796x0_SYS_CTRL1);

	if(sysVal & bq796x0_ADC_EN) {
		// SerialUSB.println("ADC Already Enabled");
	}

	// Set the ADC_EN bit
	sysVal |= bq796x0_ADC_EN;

	bq769x0_registerWrite(bq796x0_SYS_CTRL1, sysVal);

	// Enable countinous reading of the Coulomb Counter
	sysVal = bq769x0_registerRead(bq796x0_SYS_CTRL2);
	sysVal |= bq796x0_CC_EN;

	// Set the CC_EN bit
	bq769x0_registerWrite(bq796x0_SYS_CTRL2, sysVal);

	// SerialUSB.println("Coulomb counter enabled");

	// Attach interrupt
	pinMode(bq_irqPin, INPUT);

	// No pull up
	attachInterrupt(bq_irqPin, bq769x0IRQ, RISING);

	// Gain and offset are used in multiple functions
	// Read these values into global variables

	// Gain is in uV so this converts it to mV. Example: 0.370mV/LSB
	bq_gain = bq769x0_readGAIN();

	// Offset is in mV. Example: 65mV
	bq_offset = bq769x0_readADCoffset();

	// Read the system status register
	byte sysStat = bq769x0_registerRead(bq796x0_SYS_STAT);

    // Are we ready?
	if(sysStat & bq796x0_DEVICE_XREADY) {

		// SerialUSB.println("Device X Ready Error");

		// Try to clear it
		bq769x0_registerWrite(bq796x0_SYS_STAT, bq796x0_DEVICE_XREADY);

        // Datasheet does not specify how long this delay should be...
		delay(100);

		// Check again
		byte sysStat = bq769x0_registerRead(bq796x0_SYS_STAT);

		if(sysStat & bq796x0_DEVICE_XREADY) {

			// SerialUSB.println("Device X Ready Not Cleared");
            return false;
		}
	}

	return true;
}


// ---------------------------------------------------------------------
// Enable or disable the balancing of a given cell
// Give me a cell # and whether you want balancing or not
//
void enableBalancing(byte cellNumber, bool enabled) {

	byte startingBit, cellRegister;

    // Zero indexed must be convert to 1-indexed
    cellNumber ++;

	// Out of range
	if(cellNumber < 1 || cellNumber > 15) return;

	if(cellNumber < 6) {
		startingBit = 0;
		cellRegister = bq796x0_CELLBAL1;
	}
	else if(cellNumber < 11) {
		// The 2nd Cell balancing register starts at CB6
		startingBit = 6;

		// If the cell number is 6-10 then we are in the 2nd cell balancing register
		cellRegister = bq796x0_CELLBAL2;
	}
	else if(cellNumber < 16) {
		startingBit = 11;
		cellRegister = bq796x0_CELLBAL3;
	}

	// Read what is currently there
	byte cell = bq769x0_registerRead(cellRegister);

	if(enabled) {
		// Set bit for balancing
		cell |= (1<<(cellNumber - startingBit));
	}
	else {
		// Clear bit to disable balancing
		cell &= ~(1<<(cellNumber - startingBit));
	}

	// Make it so
	bq769x0_registerWrite(cellRegister, cell);
}

// ---------------------------------------------------------------------
// Calling this function will put the IC into ultra-low power SHIP mode
// A boot signal is needed to get back to NORMAL mode
//
void bq769x0_enterSHIPmode(void)
{
	byte sysValue = bq769x0_registerRead(bq796x0_SYS_CTRL1);

	// Step 1: 00
	sysValue &= 0xFC;
	bq769x0_registerWrite(bq796x0_SYS_CTRL1, sysValue);

	// Step 2: non-01
	sysValue |= 0x03;
	bq769x0_registerWrite(bq796x0_SYS_CTRL1, sysValue);

	// Step 3: 01
	sysValue &= ~(1<<1);
	bq769x0_registerWrite(bq796x0_SYS_CTRL1, sysValue);

	// Step 4: 10
	sysValue = (sysValue & 0xFC) | (1<<1);
	bq769x0_registerWrite(bq796x0_SYS_CTRL1, sysValue);

	// BQ should now be in powered down SHIP mode and will not respond to commands
	// Boot on TS1 required to start IC
}

// ---------------------------------------------------------------------
// Given a cell number, return the cell voltage
// Vcell = GAIN * ADC(cell) + OFFSET
// Conversion example from datasheet:
// 14-bit ADC = 0x1800, Gain = 0x0F, Offset = 0x1E = 2.365V
//
float bq769x0_readCellVoltage(byte cellNumber) {

	// SerialUSB.print("Read cell number: ");
	// SerialUSB.println(cellNumber);

  	if(cellNumber > 14) {
		// Return error
		return(-0);
	}

	// Reduce the caller's cell number by one so that we get register alignment
    // No longer needed since zero index now
	// cellNumber--;

	byte registerNumber = bq796x0_VC1_HI + (cellNumber * 2);

	// SerialUSB.print("register: 0x");
	// SerialUSB.println(registerNumber, HEX);

	int cellValue = bq769x0_registerDoubleRead(registerNumber);

	// int cellValue = 0x1800;  // 6,144 - Should return 2.365
	// int cellValue = 0x1F10l; // Should return 3.052

	// Cell value should now contain a 14 bit value

	// SerialUSB.print("Cell value (dec): ");
	// SerialUSB.println(cellValue);

	if(cellValue == 0) {
        return(0.0f);
    }

	// 0x1800 * 0.37 + 60 = 3,397mV
	float cellVoltage = (float) cellValue * bq_gain + bq_offset;

	cellVoltage /= 1000.0f;

	// SerialUSB.print("Cell voltage: ");
	// SerialUSB.println(cellVoltage, 3);

    cellVoltage = bq76940_roundf(cellVoltage, 2);

	return(cellVoltage);
}

// ---------------------------------------------------------------------
// Given a thermistor number return the temperature in C
// Valid thermistor numbers are 1 to 3 for external and 0 to read the internal die temp
// If you switch between internal die and external TSs this function will delay 2 seconds
//
int bq769x0_readTemp(byte thermistorNumber)
{
	// There are 3 external thermistors (optional) and an internal temp reading (channel 0)
	if(thermistorNumber < 0 || thermistorNumber > 3) {
		// Return error
		return(-0);
	}

	// SerialUSB.print("Read thermistor number: ");
	// SerialUSB.println(thermistorNumber);

	byte sysValue = bq769x0_registerRead(bq796x0_SYS_CTRL1);

	if(thermistorNumber > 0) {

		// See if we need to switch between internal die temp and external thermistor
		if((sysValue & bq796x0_TEMP_SEL) == 0) {

			// Bad news, we have to do a switch and wait 2 seconds
			// Set the TEMP_SEL bit
			sysValue |= bq796x0_TEMP_SEL;
			bq769x0_registerWrite(bq796x0_SYS_CTRL1, sysValue);

			// SerialUSB.println("Waiting 2 seconds to switch thermistors");
			delay(2000);
		}

		int registerNumber = bq796x0_TS1_HI + ((thermistorNumber - 1) * 2);
		int thermValue = bq769x0_registerDoubleRead(registerNumber);

		// Therm value should now contain a 14 bit value
		// SerialUSB.print("Therm value: 0x");
		// SerialUSB.println(thermValue, HEX);
		// 0xC89 = 3209

		// 0xC89 * 382 = 1,225,838uV. 0x233C * 382uV/LSB = 3,445,640uV
		float thermVoltage = thermValue * (float)382;
		thermVoltage /= (float)1000000;

		// Convert to V
		// SerialUSB.print("thermVoltage: ");
		// SerialUSB.println(thermVoltage, 3);

		float thermResistance = (10000.0f * thermVoltage) / (3.3f - thermVoltage);

		// SerialUSB.print("thermResistance: ");
		// SerialUSB.println(thermResistance);

		// We now have thermVoltage and resistance.
		// With a datasheet for the NTC 103AT thermistor we could
		// calculate temperature.
		int temperatureC = bq769x0_thermistorLookup(thermResistance);

		// SerialUSB.print("temperatureC: ");
		// SerialUSB.println(temperatureC);

		return(temperatureC);
	}

	else if(thermistorNumber == 0) {

		// See if we need to switch between internal die temp and external thermistor
		if((sysValue & 1<<3) != 0) {

			// Bad news, we have to do a switch and wait 2 seconds
			// Clear the TEMP_SEL bit
			sysValue &= ~(1<<3);
			bq769x0_registerWrite(bq796x0_SYS_CTRL1, sysValue);

			// SerialUSB.println("Waiting 2 seconds to switch to internal die thermistors");
			delay(2000);
		}

		// There are multiple internal die temperatures. We are only going to grab 1.
		int thermValue = bq769x0_registerDoubleRead(bq796x0_TS1_HI);

		// Therm value should now contain a 14 bit value
		// SerialUSB.print("Therm value: 0x");
		// SerialUSB.println(thermValue, HEX);

		// 0xC89 * 382 = 1,225,838uV. 0x233C * 382uV/LSB = 3,445,640uV
		float thermVoltage = thermValue * (float)382;

		// Convert to V
		thermVoltage /= (float)1000000;

		// SerialUSB.print("thermVoltage: ");
		// SerialUSB.println(thermVoltage, 3);

		float temperatureC = 25.0f - ((thermVoltage - 1.2f) / 0.0042f);

		// SerialUSB.print("temperatureC: ");
		// SerialUSB.println(temperatureC);

		return((int)temperatureC);
	}

    return(-0);
}

// ---------------------------------------------------------------------
// Returns the coulomb counter value in microVolts
// Example: 84,400uV
// Coulomb counter is enabled during bqInit(). We do not use oneshot.
// If the counter is enabled in ALWAYS ON mode it will set the ALERT pin every 250ms.
// You can respond to this however you want.
// Host may clear the CC_READY bit or let it stay at 1.
//
float bq769x0_readCoulombCounter(void) {

	int count = bq769x0_registerDoubleRead(bq796x0_CC_HI);

	// int count = 0xC350; //Test. Should report -131,123.84

	// count should be naturally in 2's compliment. count_uV is now in uV
	float count_uV = count * 8.44f;

	return(count_uV);
}

// ---------------------------------------------------------------------
// Returns the pack voltage in volts
// Vbat = 4 * GAIN * ADC(cell) + (# of cells * bq_offset)
//
float bq769x0_readPackVoltage(void) {

	unsigned int packADC = bq769x0_registerDoubleRead(bq796x0_BAT_HI);

	// SerialUSB.print("packADC = ");
	// SerialUSB.println(packADC);

	// packADC = 0x6DDA; //28,122 Test. Should report something like 42.520V

	// packADC = 35507
	// bq_gain = 0.38uV/LSB
	// bq_offset = 47mV
	// 53970

	// 53970 in uV?
	float packVoltage = 4 * bq_gain * packADC;

	// Should be in mV
	packVoltage += (NUMBER_OF_CELLS * bq_offset);

	// Convert to volts
	return(packVoltage / (float)1000);
}

// ---------------------------------------------------------------------
// Reads the bq_gain registers and calculates the system's factory trimmed bq_gain
// GAIN = 365uV/LSB + (ADCGAIN<4:0>) * 1uV/LSB
// ADC bq_gain comes from two registers that have to be moved around and combined.
//
float bq769x0_readGAIN(void) {

  	byte val1 = bq769x0_registerRead(bq796x0_ADCGAIN1);
  	byte val2 = bq769x0_registerRead(bq796x0_ADCGAIN2);

	// There are some unknown reservred bits around val1 that need to be cleared
	val1 &= 0b00001100;

	// Recombine the bits into one ADCGAIN
	byte adcGain = (val1 << 1) | (val2 >> 5);

	float gain = (365 + adcGain) / 1000.0f;

	return(gain);
}

// ---------------------------------------------------------------------
// Returns the factory trimmed ADC bq_offset
// Offset is -127 to 128 in mV
//
int bq769x0_readADCoffset(void) {

  	// Here we need to convert a 8bit 2's compliment to a 16 bit int
  	char offset = bq769x0_registerRead(bq796x0_ADCOFFSET);

	// 8 bit char is now a full 16-bit int. Easier math later on.
  	return((int)offset);
}

// ---------------------------------------------------------------------
// Returns the over voltage trip threshold
// Default is 0b.10.OVTRIP(0xAC).1000 = 0b.10.1010.1100.1000 = 0x2AC8 = 10,952
// OverVoltage = (OV_TRIP * GAIN) + ADCOFFSET
// Gain and Offset is different for each IC
// Example: voltage = (10,952 * 0.370) + 56mV = 4.108V
//
float bq769x0_readOVtrip(void) {

  	int trip = bq769x0_registerRead(bq796x0_OV_TRIP);

	// Shuffle the bits to align to 0b.10.XXXX.XXXX.1000
  	trip <<= 4;
  	trip |= 0x2008;

	float overVoltage = ((float)trip * bq_gain) + bq_offset;

	// Convert to volts
	overVoltage /= 1000;

	// SerialUSB.print("overVoltage should be around 4.108: ");
	// SerialUSB.println(overVoltage, 3);

	return(overVoltage);
}

// ---------------------------------------------------------------------
// Given a voltage (4.22 for example), set the over voltage trip register
// Example: voltage = 4.2V = (4200mV - 56mV) / 0.370mv = 11,200
// 11,200 = 0x2BC0 =
//
void bq769x0_writeOVtrip(float tripVoltage) {

	// Convert voltage to an 8-bit middle value
  	byte val = bq769x0_tripCalculator(tripVoltage);

  	bq769x0_registerWrite(bq796x0_OV_TRIP, val);
}

// ---------------------------------------------------------------------
// Returns the under voltage trip threshold
// Default is 0b.01.UVTRIP(0x97).0000 = 0x1970 = 6,512
// UnderVoltage = (UV_TRIP * GAIN) + ADCOFFSET
// Gain and Offset is different for each IC
// Example: voltage = (6,512 * 0.370) + 56mV = 2.465V
//
float bq769x0_readUVtrip(void) {

  	int trip = bq769x0_registerRead(bq796x0_UV_TRIP);

	// Shuffle the bits to align to 0b.01.XXXX.XXXX.0000
  	trip <<= 4;
  	trip |= 0x1000;

	// Convert to volts
  	float underVoltage = ((float)trip * bq_gain) + bq_offset;
  	underVoltage /= 1000;

  	// SerialUSB.print("underVoltage should be around 2.465: ");
  	// SerialUSB.println(underVoltage, 3);

  	return(underVoltage);
}

// ---------------------------------------------------------------------
// Given a voltage (2.85V for example), set the under voltage trip register
//
void bq769x0_writeUVtrip(float tripVoltage) {

	// Convert voltage to an 8-bit middle value
	byte val = bq769x0_tripCalculator(tripVoltage);

	// address, value
	bq769x0_registerWrite(bq796x0_UV_TRIP, val);
}

// ---------------------------------------------------------------------
// Under voltage and over voltage use the same rules for calculating the 8-bit value
// Given a voltage this function uses bq_gain and bq_offset to get a 14 bit value
// Then strips that value down to the middle-ish 8-bits
// No registers are written, that's up to the caller
//
byte bq769x0_tripCalculator(float tripVoltage) {

	// Convert volts to mV
  	tripVoltage *= 1000;

	// SerialUSB.print("tripVoltage to be: ");
	// SerialUSB.println(tripVoltage, 3);

	tripVoltage -= bq_offset;
	tripVoltage /= bq_gain;

	// We only want the integer - drop decimal portion.
	int tripValue = (int)tripVoltage;

	// SerialUSB.print("tripValue should be something like 0x2BC0: ");
	// SerialUSB.println(tripValue, HEX);

	// Cut off lower 4 bits
	tripValue >>= 4;

	// Cut off higher bits
	tripValue &= 0x00FF;

	// SerialUSB.print("About to report tripValue: ");
	// SerialUSB.println(tripValue, HEX);

	return(tripValue);
}

// ---------------------------------------------------------------------
// Write a given value to a given register
//
void bq769x0_registerWrite(byte regAddress, byte regData) {

	bqWire.beginTransmission(bq_I2CAddress);
	bqWire.write(regAddress);
	bqWire.endTransmission();

	bqWire.beginTransmission(bq_I2CAddress);
	bqWire.write(regAddress);
	bqWire.write(regData);
	bqWire.endTransmission();
}

// #define DEBUG_RW

// ---------------------------------------------------------------------
// Returns a given register
//
byte bq769x0_registerRead(byte regAddress) {

    #ifdef DEBUG_RW
	SerialUSB.print("Select Register: ");
	SerialUSB.print(regAddress);
	SerialUSB.print(" on device: ");
	SerialUSB.println(bq_I2CAddress);
    #endif

	bqWire.beginTransmission(bq_I2CAddress);
	bqWire.write(regAddress);
	byte status = bqWire.endTransmission();

	if(status != 0) {
		SerialUSB.println("NACK");
		return(0);
	}

    #ifdef DEBUG_RW
	SerialUSB.print("Request 1b from device: ");
	SerialUSB.println(bq_I2CAddress);
    #endif

	// Here's where I2C can time out
	bqWire.requestFrom(bq_I2CAddress, 1, false);

	byte counter = 0;

    byte i2c_registers[1] = {0};
	
    while(bqWire.available() == 0) {
		if(counter++ > MAX_I2C_TIME) {

			// Timeout, no good
            #ifdef DEBUG_RW
            SerialUSB.println("timeout");
            #endif 

            // Return with error
			return(0);
		}
        delay(1);
	}

	i2c_registers[0] = bqWire.read();

    #ifdef DEBUG_RW
	SerialUSB.print("Response: 0x");
	SerialUSB.println(i2c_registers[0], HEX);
	#endif

	return(i2c_registers[0]);
}

// ---------------------------------------------------------------------
// Returns the atmoic int from two sequentials reads
//
int bq769x0_registerDoubleRead(byte regAddress) {

	#ifdef DEBUG_RW
	SerialUSB.print("Select Register: ");
	SerialUSB.print(regAddress);
	SerialUSB.print(" on device: ");
	SerialUSB.println(bq_I2CAddress);
	#endif

	bqWire.beginTransmission(bq_I2CAddress);
	bqWire.write(regAddress);
    bqWire.endTransmission();

	#ifdef DEBUG_RW
	SerialUSB.print("Request 2b from: ");
	SerialUSB.println(bq_I2CAddress);
	#endif

    byte i2c_registers[2] = {0};

	// Here's where I2C can time out...
    for(uint8_t bytes = 0; bytes < 2; bytes ++) {

        bqWire.requestFrom(bq_I2CAddress, 1, true);

        byte counter = 0;

        while(bqWire.available() < 1) {
            if(counter++ > MAX_I2C_TIME) {
                
                // Timeout, no good...
            	#ifdef DEBUG_RW
                SerialUSB.println("timeout");
                #endif

                // Return with error
                return(-1); 
            }
            delay(1);
        }
    
        i2c_registers[bytes] = bqWire.read();
    }

	#ifdef DEBUG_RW
	SerialUSB.print("reg1: 0x");
	SerialUSB.print(i2c_registers[0], HEX);
	SerialUSB.print(" reg2: 0x");
	SerialUSB.println(i2c_registers[1], HEX);
	#endif

	int sixteen_bit_word = (int)i2c_registers[0] << 8;
	sixteen_bit_word |= i2c_registers[0];

	return(sixteen_bit_word);
}

// ---------------------------------------------------------------------
// Given a resistance on a super common 103AT-2 thermistor, return a temperature in C
// This is a poor way of converting the resistance to temp but it works for now
// From: http://www.rapidonline.com/pdf/61-0500e.pdf
//
int bq769x0_thermistorLookup(float resistance) {

	// Resistance is coming in as Ohms, this lookup table assume kOhm

	// Convert to kOhm
	resistance /= 1000;

	int temp = 0;

	if(resistance > 329.5f) temp = -50;
	if(resistance > 247.7f) temp = -45;
	if(resistance > 188.5f) temp = -40;
	if(resistance > 144.1f) temp = -35;
	if(resistance > 111.3f) temp = -30;
	if(resistance > 86.43f) temp = -25;
	if(resistance > 67.77f) temp = -20;
	if(resistance > 53.41f) temp = -15;
	if(resistance > 42.47f) temp = -10;
	if(resistance > 33.90f) temp = -5;
	if(resistance > 27.28f) temp = 0;
	if(resistance > 22.05f) temp = 5;
	if(resistance > 17.96f) temp = 10;
	if(resistance > 14.69f) temp = 15;
	if(resistance > 12.09f) temp = 20;
	if(resistance > 10.00f) temp = 25;
	if(resistance > 8.313f) temp = 30;

	return(temp);
}

// ---------------------------------------------------------------------
// Wake BQ with boot signal on TS1
//
void bq769x0_wake(){
	pinMode(PIN_BQ_TS1, OUTPUT);
	digitalWrite(PIN_BQ_TS1, HIGH);
	delay(2);
	digitalWrite(PIN_BQ_TS1, LOW);
	pinMode(PIN_BQ_TS1, INPUT);
	delay(10);
}

// ---------------------------------------------------------------------
// Reset BQ
//
void bq769x0_reset() {
	pinMode(PIN_BQ_RESET, OUTPUT);
	digitalWrite(PIN_BQ_RESET, HIGH);
	delay(2);
	digitalWrite(PIN_BQ_RESET, LOW);
}