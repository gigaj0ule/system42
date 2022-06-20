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

            // Config
            int32_t SM72442_tare_Vout   = 0;
            int32_t SM72442_tare_Iout   = 0;
            int32_t SM72442_tare_Vin    = 0;
            int32_t SM72442_tare_Iin    = 0;


            auto customized_volatile_objects() {
                return make_protocol_member_list(

                    // Register 0
                    make_protocol_object("adc", 
                        make_protocol_number_kw(
                            &SM72242_adc6,
                            property_name = "adc6",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72242_adc4,
                            property_name = "adc4",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72242_adc2,
                            property_name = "adc2",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72242_adc0,
                            property_name = "adc0",
                            property_is_read_only = true
                        )
                    ),

                    // Register 1
                    make_protocol_object("iv",
                        make_protocol_number_kw(
                            &SM72242_vout,
                            property_name = "vout",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72242_iout,
                            property_name = "iout",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72242_vin,
                            property_name = "vin",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72242_iin,
                            property_name = "iin",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = true
                        )
                    ),

                    // Register 4
                    make_protocol_object("config",
                        make_protocol_number_kw(
                            &SM72242_override_adcprog,
                            property_name = "or_adc",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_power_thresh_select,
                            property_name = "p_th_sel",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_bb_in_ptmode_sel,
                            property_name = "bb_pt_mode_sel",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_iout_max,
                            property_name = "iout_max",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_vout_max,
                            property_name = "vout_max",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_deadtime_off,
                            property_name = "dt_off",
                            property_min_value = 0,
                            property_max_value = 7,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_deadtime_on,
                            property_name = "dt_on",
                            property_min_value = 0,
                            property_max_value = 7,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_duty_cycle_open,
                            property_name = "duty_open",
                            property_min_value = 0,
                            property_max_value = 511,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_pass_thru_select,
                            property_name = "passthru_sel",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_pass_thru_manual,
                            property_name = "passthru_man",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_soft_reset,
                            property_name = "reset",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_pll_clock,
                            property_name = "pll_clk",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_open_loop_enable,
                            property_name = "ol_en",
                            property_is_read_only = false
                        )
                    ),

                    // Register 4
                    make_protocol_object("offset",
                        make_protocol_number_kw(
                            &SM72242_vout_offset,
                            property_name = "vout_os",
                            property_min_value = 0,
                            property_max_value = 255,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_iout_offset,
                            property_name = "iout_os",
                            property_min_value = 0,
                            property_max_value = 255,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_vin_offset,
                            property_name = "vin_os",
                            property_min_value = 0,
                            property_max_value = 255,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_iin_offset,
                            property_name = "iin_os",
                            property_min_value = 0,
                            property_max_value = 255,
                            property_is_read_only = false
                        )
                    ),

                    // Register 5
                    make_protocol_object("thresholds",
                        make_protocol_number_kw(
                            &SM72242_iin_hi_th,
                            property_name = "iin_hi",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_iin_lo_th,
                            property_name = "iin_lo",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_iout_hi_th,
                            property_name = "iout_hi",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &SM72242_iout_lo_th,
                            property_name = "iout_lo",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = false
                        )
                    ),

                    make_protocol_object("tare",
                        make_protocol_number_kw(
                            &SM72442_tare_Vout,
                            property_name = "tare_vout",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72442_tare_Iout,
                            property_name = "tare_iout",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72442_tare_Vin,
                            property_name = "tare_vin",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &SM72442_tare_Iin,
                            property_name = "tare_iin",
                            property_min_value = 0,
                            property_max_value = 1023,
                            property_is_read_only = true
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

            auto customized_interrupts() {
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
