#ifndef __SKETCH_HPP_
    #define __SKETCH_HPP_

    #ifdef USE_BITSNAP
    #include "communication.h"

    //void remote_update_occured(void *);

    static void host_wrote_data_callback(void *) {
        // Stuff
    }

    static void host_read_data_callback(void *) {
        // Stuff
    }

    // See README.md for information about how to construct 
    // this class

    class communication {

        public:
            char device_model[16]  = "stm42";

            // Register 0
            int32_t SM72242_adc0    = 0;
            int32_t SM72242_adc2    = 0;
            int32_t SM72242_adc4    = 0;
            int32_t SM72242_adc6    = 0;

            // Register 1
            int32_t SM72242_vout    = 0;
            int32_t SM72242_iout    = 0;
            int32_t SM72242_vin     = 0;
            int32_t SM72242_iin     = 0;

            // Register 3
            bool        SM72242_override_adcprog    = 0;
            bool        SM72242_power_thresh_select = 0;
            uint8_t     SM72242_bb_in_ptmode_sel    = 0;
            int32_t     SM72242_iout_max            = 0;
            int32_t     SM72242_vout_max            = 0;
            uint8_t     SM72242_deadtime_off        = 0;
            uint8_t     SM72242_deadtime_on         = 0;
            int32_t     SM72242_duty_cycle_open     = 0;
            bool        SM72242_pass_thru_select    = 0;
            bool        SM72242_pass_thru_manual    = 0;
            bool        SM72242_soft_reset          = 0;
            bool        SM72242_pll_clock           = 0;
            bool        SM72242_open_loop_enable    = 0;

            // Register 4
            uint8_t SM72242_vout_offset = 0;
            uint8_t SM72242_iout_offset = 0;
            uint8_t SM72242_vin_offset  = 0;
            uint8_t SM72242_iin_offset  = 0;

            // Register 5
            int32_t SM72242_iin_hi_th   = 0;
            int32_t SM72242_iin_lo_th   = 0;
            int32_t SM72242_iout_hi_th  = 0;
            int32_t SM72242_iout_lo_th  = 0;


            auto customized_volatile_objects() {
                return make_protocol_member_list(

                    // Register 0
                    make_protocol_object("ADC", 
                        make_protocol_number_kw(
                            &SM72242_adc6,
                            property_name = "ADC6",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72242_adc4,
                            property_name = "ADC4",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72242_adc2,
                            property_name = "ADC2",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72242_adc0,
                            property_name = "ADC0",
                            property_is_read_only = true
                        )
                    ),

                    // Register 1
                    make_protocol_object("IV",
                        make_protocol_number_kw(
                            &SM72242_vout,
                            property_name = "VOUT",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72242_iout,
                            property_name = "IOUT",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72242_vin,
                            property_name = "VIN",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72242_iin,
                            property_name = "IIN",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = true
                        )
                    ),

                    // Register 4
                    make_protocol_object("CONFIG",
                        make_protocol_number_kw(
                            &SM72242_override_adcprog,
                            property_name = "OR_ADC",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_power_thresh_select,
                            property_name = "P_TH_SEL",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_bb_in_ptmode_sel,
                            property_name = "BB_PT_MODE_SEL",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_iout_max,
                            property_name = "IOUT_MAX",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_vout_max,
                            property_name = "VOUT_MAX",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_deadtime_off,
                            property_name = "DT_OFF",
                            property_min_value = 0,
                            property_max_value = 7,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_deadtime_on,
                            property_name = "DT_ON",
                            property_min_value = 0,
                            property_max_value = 7,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_duty_cycle_open,
                            property_name = "DUTY_OPEN",
                            property_min_value = 0,
                            property_max_value = 511,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_pass_thru_select,
                            property_name = "PASSTRHU_SEL",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_pass_thru_manual,
                            property_name = "PASSTHRU_MAN",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_soft_reset,
                            property_name = "RESET",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_pll_clock,
                            property_name = "PLL_CLK",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_open_loop_enable,
                            property_name = "OL_EN",
                            property_is_read_only = false
                        )
                    ),

                    // Register 4
                    make_protocol_object("OFFSET",
                        make_protocol_number_kw(
                            &SM72242_vout_offset,
                            property_name = "VOUT_OS",
                            property_min_value = 0,
                            property_max_value = 255,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_iout_offset,
                            property_name = "IOUT_OS",
                            property_min_value = 0,
                            property_max_value = 255,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_vin_offset,
                            property_name = "VIN_OS",
                            property_min_value = 0,
                            property_max_value = 255,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_iin_offset,
                            property_name = "IIN_OS",
                            property_min_value = 0,
                            property_max_value = 255,
                            property_is_read_only = false
                        )
                    ),

                    // Register 5
                    make_protocol_object("THRESHOLDS",
                        make_protocol_number_kw(
                            &SM72242_iin_hi_th,
                            property_name = "IIN_HI",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_iin_lo_th,
                            property_name = "IIN_LO",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_iout_hi_th,
                            property_name = "IOUT_HI",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_iout_lo_th,
                            property_name = "IOUT_LO",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = false
                        )
                    )
                );
            };

            struct NvmProperties_t {
                int32_t nv_property = 0;
            } non_volatile_properties;

            auto customized_nonvolatile_objects() {
                return make_protocol_member_list();
            }

            event_vector_t scan_complete_event = {1};

            auto customized_event_triggers() {
                return make_protocol_member_list(

                    make_event_trigger(
                        &scan_complete_event,
                        property_name = "scan_complete"
                    )
                );
            };
        };
    #endif

#endif
