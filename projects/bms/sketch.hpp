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
            char device_make[16] = "i2c_scan";
            char device_model[16]  = "stm42";

            char i2c_addresses[127*6] = {0};

            int32_t reg_staus = 0;

            char i2c_states[5][16] = {
                "SUCCESS",
                "OVERFLOW",
                "NACK ADDR",
                "NACK DATA",
                "MISC ERROR"
            };

            auto customized_volatile_objects() {
                return make_protocol_member_list(
                    make_protocol_number_kw(
                        &reg_staus,
                        property_name = "reg_staus",
                        property_is_read_only = true
                    )
                    
                    /*,

                    make_protocol_string_kw(
                        &i2c_addresses, 
                        property_name = "i2c_addresses", 
                        property_length = sizeof(i2c_addresses),
                        property_is_read_only = true
                    )*/
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
