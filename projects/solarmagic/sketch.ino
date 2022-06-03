// Headers
#include "sketch.hpp"
#include "sketch_only_statics.hpp"
#include "Wire.h"

#include "protocol.hpp"

#include "sm72242.h"

#if defined(USBD_USE_HID_COMPOSITE)
    #include "Keyboard.h"
    #include "Mouse.h"
#endif


#define PIN_I2C_SCL1 PB6
#define PIN_I2C_SDA1 PB7
#define ADDRESS_SM72442 0x01

TwoWire I2C_BUS (PIN_I2C_SDA1, PIN_I2C_SCL1);

void find_i2c_devices();


static void worker_thread(void* arg) {

	// Read initial config state
	SM72442_STATE_SYNC_config();
	communicable.SM72242_override_adcprog		= SM72242_STATE_override_adcprog;
	communicable.SM72242_power_thresh_select    = SM72242_STATE_power_thresh_select;
	communicable.SM72242_bb_in_ptmode_sel       = SM72242_STATE_bb_in_ptmode_sel;
	communicable.SM72242_iout_max               = SM72242_STATE_iout_max;
	communicable.SM72242_vout_max               = SM72242_STATE_vout_max;
	communicable.SM72242_deadtime_off           = SM72242_STATE_deadtime_off;
	communicable.SM72242_deadtime_on 			= SM72242_STATE_deadtime_on;
	communicable.SM72242_duty_cycle_open        = SM72242_STATE_duty_cycle_open;
	communicable.SM72242_pass_thru_select       = SM72242_STATE_pass_thru_select;
	communicable.SM72242_pass_thru_manual       = SM72242_STATE_pass_thru_manual;
	communicable.SM72242_soft_reset				= SM72242_STATE_soft_reset;
	communicable.SM72242_pll_clock				= SM72242_STATE_pll_clock;
	communicable.SM72242_open_loop_enable		= SM72242_STATE_open_loop_enable;

    while(true) {
	    
		// SM72242 Loop Task
        os_delay(1000);

		// Register 0
		SM72442_readADC(
			&communicable.SM72242_adc6, 
			&communicable.SM72242_adc4, 
			&communicable.SM72242_adc2, 
			&communicable.SM72242_adc0
		);

		// Register 1
		SM72442_readIV(
			&communicable.SM72242_vout, 
			&communicable.SM72242_iout, 
			&communicable.SM72242_vin, 
			&communicable.SM72242_iin
		);

		// Check config state
		SM72442_STATE_SYNC_config();

		// If communicable(desired) state changed, update SM72442 state
		if(communicable.SM72242_override_adcprog != SM72242_STATE_override_adcprog) {
			SM72442_setConfig_override_adcprog(communicable.SM72242_override_adcprog);
		}
		if(communicable.SM72242_iout_max != SM72242_STATE_iout_max) {
			SM72442_setConfig_iout_max(communicable.SM72242_iout_max);
		}
		if(communicable.SM72242_vout_max != SM72242_STATE_vout_max) {
			SM72442_setConfig_vout_max(communicable.SM72242_vout_max);
		}
		if(communicable.SM72242_deadtime_off != SM72242_STATE_deadtime_off) {
			SM72442_setConfig_deadtime_off(communicable.SM72242_deadtime_off);
		}
		if(communicable.SM72242_deadtime_on != SM72242_STATE_deadtime_on) {
			SM72442_setConfig_deadtime_on(communicable.SM72242_deadtime_on);
		}
		if(communicable.SM72242_soft_reset != SM72242_STATE_soft_reset) {
			SM72442_setConfig_override_soft_reset(communicable.SM72242_soft_reset);
		}

		// Todo: Verify if state was updated
		SM72442_readConfig(
			&communicable.SM72242_override_adcprog, 
			&communicable.SM72242_power_thresh_select,
			&communicable.SM72242_bb_in_ptmode_sel,
			&communicable.SM72242_iout_max,
			&communicable.SM72242_vout_max,
			&communicable.SM72242_deadtime_off,
			&communicable.SM72242_deadtime_on,
			&communicable.SM72242_duty_cycle_open,
			&communicable.SM72242_pass_thru_select, 
			&communicable.SM72242_pass_thru_manual,
			&communicable.SM72242_soft_reset,
			&communicable.SM72242_pll_clock,
			&communicable.SM72242_open_loop_enable
		);

		// Register 4
		SM72442_readOffsets(
			&communicable.SM72242_vout_offset, 
			&communicable.SM72242_iout_offset, 
			&communicable.SM72242_vin_offset, 
			&communicable.SM72242_iin_offset
		);

		// Register 5
		SM72442_readCurrentThresholds(
			&communicable.SM72242_iin_hi_th,
			&communicable.SM72242_iin_lo_th,
			&communicable.SM72242_iout_hi_th,
			&communicable.SM72242_iout_lo_th
		);


		// Tare
		//SM72442_calibrateCurrentOffsets(os_delay);
    }
}

void setup() {

    #if defined(USBD_USE_HID_COMPOSITE)
        Mouse.begin();
        Keyboard.begin();
    #endif
        
    // Init communication
    early_setup();

	I2C_BUS.begin();
	I2C_BUS.setClock(100000);

    // Launch program!
    create_threads(worker_thread);
};


void loop(){    
    __asm__ volatile("nop");
    //os_delay(1);
    
    #if defined(USBD_USE_CDC)
    //SerialUSB.print("Hi");
    #endif
};
