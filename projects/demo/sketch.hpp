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
            char device_model[16]  = "stm32f103";

            bool bool_property = false;
            uint8_t uint8_property = 255;
            int32_t int32_property = 2147483647;
            int64_t int64_property = 223372036854775807;
            float float_property = 80.0f;
            char str_property[1024] = "Hello world";
            char buffer_property[60] = "DEADBEEF";

            auto customized_volatile_objects() {
                return make_protocol_member_list(

                    make_protocol_number_kw(
                        &uint8_property,
                        property_name = "uint8_property",
                        after_written_callback = &host_wrote_data_callback,
                        after_read_callback = &host_read_data_callback
                    ),

                    make_protocol_number_kw(
                        &bool_property,
                        property_name = "bool_property"
                    ),

                    make_protocol_number_kw(
                        &int32_property,
                        property_name = "int32_property"
                    ),

                    make_protocol_number_kw(
                        &int64_property,
                        property_name = "int64_property",
                        property_min_value = (int64_t)(-20), 
                        property_max_value = (int64_t)20
                    ),

                    make_protocol_number_kw(
                        &float_property,
                        property_name = "float_property"
                    ),

                    make_protocol_string_kw(
                        &str_property, 
                        property_name = "string_property",
                        property_length = sizeof(str_property)
                    ),

                    make_protocol_buffer_kw(
                        &buffer_property, 
                        property_name = "buffer", 
                        property_length = sizeof(buffer_property)
                    )

                );
            };

            struct NvmProperties_t {
                int32_t nv_property = 0;
            } non_volatile_properties;

            auto customized_nonvolatile_objects() {
                return make_protocol_member_list(
                    make_protocol_number_kw(
                        &non_volatile_properties.nv_property,
                        property_name = "sample_nv_property"
                    )
                );
            }

            event_vector_t event_trigger = {1};

            auto customized_interrupts() {
                return make_protocol_member_list(

                    make_event_trigger(
                        &event_trigger,
                        property_name = "sample_event"
                    )
                );
            };
        };
    #endif

#endif
