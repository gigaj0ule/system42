// Includes
#include "bms_functions.h"
#include "SparkFun_bq769x0.h"

// Communication
#include "powernet.h"
#include "sketch.hpp"
extern PowernetNamespace pntp;


bool enabled_adc_channels[15] = {
	true,   // 0
	true,   // 1
	true,   // 2
	false,  // 3
	true,	// 4
	true, 	// 5
	true,  	// 6
	true,	// 7
	false, 	// 8
	true,	// 9
	true,	// 10
	true,	// 11
	true, 	// 12
	false,	// 13
	true	// 14
}


// ----------------------------------------------------
// Write to the data bus 
// 
void data_bus_write (uint8_t address) {

	pinMode(PIN_DATA_BUS_0, OUTPUT);
	pinMode(PIN_DATA_BUS_1, OUTPUT);
	pinMode(PIN_DATA_BUS_2, OUTPUT);
	pinMode(PIN_DATA_BUS_3, OUTPUT);

	bool bit_0 = (address & 0b0001);
	bool bit_1 = (address & 0b0010);
	bool bit_2 = (address & 0b0100);
	bool bit_3 = (address & 0b1000);

	digitalWrite(PIN_DATA_BUS_0, bit_0);
	digitalWrite(PIN_DATA_BUS_1, bit_1);
	digitalWrite(PIN_DATA_BUS_2, bit_2);
	digitalWrite(PIN_DATA_BUS_3, bit_3);
}

// ----------------------------------------------------
// Measure a thermistor channel
//
// Thermistor parameters:
#define TS_RT0 10000   // Ω
#define TS_B 3977      // K
#define TS_VCC 3.3    	// Supply voltage
#define TS_R 10000  	// R=10KΩ

float bq_measureThermistorChannel(byte channel) {

	// Set ADC to input
	pinMode(PIN_TS_IN, INPUT);

	// Address appropriate thermistor channel
	data_bus_write(channel);

	// Settle
	delayMicroseconds(10);

	// Acquisition analog value of VRT
	float vrt = analogRead(PIN_TS_IN);

	// Temperature T0 from datasheet, conversion from Celsius to kelvin
	float t0 = 25.0f + 273.15f;                 
	
	// Conversion to voltage
	vrt = (TS_VCC / 1023.00f) * vrt;      
	float vr = TS_VCC - vrt;

	// Resistance of RT
	float rt = vrt / (vr / TS_R);               
	float ln = log(rt / TS_RT0);

	// Temperature from thermistor
	float tx = (1.0f / ((ln / TS_B) + (1.0f / t0))); 

	// Conversion to Celsius
	tx -= 273.15f;                 

	// Return temperature
	return tx;
}


// ----------------------------------------------------
// Measure all thermistors
//
#define NUMBER_OF_THERMISTORS 12

void bq_readCellTemps(void) {

	// We multiplex the thermistors by having two
	// analog grounds

	// Set ground select as output
	pinMode(PIN_TS_A_B, OUTPUT);

	// Select ground A
	digitalWrite(PIN_TS_A_B, LOW);

	// Wait to settle
	delay(1);

	// Read channels on A
	for(int i = 0 ; i < NUMBER_OF_THERMISTORS ; i++) {
		pntp.b0_cell_temperatue[0][i] = bq_measureThermistorChannel(i);
	}

	// Select ground B
	digitalWrite(PIN_TS_A_B, HIGH);

	// Wait to settle
	delay(1);

	// Read channels on B
	for(int i = 0 ; i < NUMBER_OF_THERMISTORS ; i++) {
		pntp.b0_cell_temperatue[1][i] = bq_measureThermistorChannel(i);
	}

	// Reset ground state
	digitalWrite(PIN_TS_A_B, LOW);
}


// ----------------------------------------------------
// Measure all cell V
//

// Cell voltage accumulator
float cell_voltage_accumulator;
float cell_voltage_min;
float cell_voltage_max;

void bq_readCellVoltages(void){
	
	// Cell statistics reset
	cell_voltage_accumulator = 0.0f;
	cell_voltage_min = 0.0f;
	cell_voltage_max = 0.0f;
	
	int num_cells = 0;

	// Iterate among cells
	for(int i = 0 ; i < NUMBER_OF_CELLS ; i++) {

		if(enabled_adc_channels[i] == false) {
			return;
		}

		// Count this cell
		num_cells ++;

		// Read cell voltage
		pntp.b0_cell_voltage[i] = bq769x0_readCellVoltage(i);
	
		// Sum voltages
		cell_voltage_accumulator += pntp.b0_cell_voltage[i];

		// What's the min?
		if(cell_voltage_min > pntp.b0_cell_voltage[i]) {
			cell_voltage_min = pntp.b0_cell_voltage[i];
		};

		// What's the max?
		if(cell_voltage_max < pntp.b0_cell_voltage[i]) {
			cell_voltage_max = pntp.b0_cell_voltage[i];
		};
	}

	// Calculate mean cell voltage
	pntp.b0_cell_voltage_mean = cell_voltage_accumulator / (float) num_cells;
}


// ----------------------------------------------------
// Balance pack
//
// This function does not call bq_readCellVoltages()
// and bq_readCellVoltages() must be called before it
// or balancing will not be accurate
//
bool balance_flip_flop = false;
float cell_voltage_tolerance = 0.1f;

void bq_balanceCells(void){

	// First disable all balanced cells
	for(int i = 0 ; i < NUMBER_OF_CELLS ; i++) {

		// Zero voltage cells should not be balanced or counted
		if(enabled_adc_channels[i] == false) {
			return;
		}

		// Disable this particular cell [i]
		bq769x0_enableBalancing(i, false);

		// Make sure the IC die temperature isn't too high.
		// If it is then we cannot continue
		for(int i = 0; i < 3; i++) {
			pntp.b0_adc_temp[i] = bq769x0_readDieTemp(i);
		}

		// 50*C seems like a good limit
		// any hotter and finger will burn
		if(pntp.b0_adc_temp > 50.0f) {
			// Too hot, must cool down
			return;
		}

		// Now we calculate what cells need balancing
		// by comparing the desired balance tolerance 
		// to the mean
		float cell_voltage_difference = pntp.b0_cell_voltage[i] - pntp.b0_cell_voltage_mean;

		// Balance this cell if the tolerance is exceeded
		if(cell_voltage_difference > cell_voltage_tolerance) {

			// The bq data sheet says we cannot balance 2 adjacent 
			// cells. So, this function needs memory. 

			// On every other function call flip_flop flops
			balance_flip_flop = !balance_flip_flop;
			
			// Cells 
			bool cell_is_odd = i % 2;

			// Balance odd cells on flip_flop 1
			if(cell_is_odd & balance_flip_flop) {
				bq769x0_enableBalancing(i, true);
			}

			// Balance even cells on flip_flop 0
			if(!cell_is_odd & !balance_flip_flop) {
				bq769x0_enableBalancing(i, true);				
			}
		}
	}
}