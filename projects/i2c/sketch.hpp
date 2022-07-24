#ifndef __SKETCH_HPP_
    #define __SKETCH_HPP_

    #ifdef INCLUDE_PNTP

    class PowernetNamespace {

        public:

            char i2c_addresses[127*6] = {0};

            uint8_t i2c_device_count = 0;

            bool afe_reset = 0;
            bool afe_wake = 0;

            char i2c_states[5][16] = {
                "SUCCESS",
                "OVERFLOW",
                "NACK ADDR",
                "NACK DATA",
                "MISC ERROR"
            };

            auto communicable_variables() {
                return make_protocol_member_list(
                    make_protocol_number_kw(
                        &afe_wake,
                        property_name = "afe_wake",
                        property_is_read_only = false
                    ),

                    make_protocol_number_kw(
                        &afe_reset,
                        property_name = "afe_reset",
                        property_is_read_only = false
                    ),

                    make_protocol_number_kw(
                        &i2c_device_count,
                        property_name = "i2c_device_count",
                        property_is_read_only = true
                    ),

                    make_protocol_string_kw(
                        &i2c_addresses, 
                        property_name = "i2c_addresses", 
                        property_length = sizeof(i2c_addresses),
                        property_is_read_only = true
                    )
                );

            };

            struct DiskStructure_t {
                int32_t nv_property = 0;
            } disk0;

            auto non_communicable_variables() {
                return make_protocol_member_list();
            }

            event_vector_t scan_complete_event = {1};

            auto communicable_interrupts() {
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
