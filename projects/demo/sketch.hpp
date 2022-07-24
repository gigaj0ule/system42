#ifndef __SKETCH_HPP_
    #define __SKETCH_HPP_

    #ifdef INCLUDE_PNTP
    #include "powernet.h"


    static void host_wrote_data_callback(void *) {
        // Stuff
    }

    static void host_read_data_callback(void *) {
        // Stuff
    }


    class PowernetNamespace {

        public:

            void demo_func_with_args(float arg_1);

            bool bool_property = false;
            int32_t int32_property = 2147483647;
            int64_t int64_property = 223372036854775807;
            float float_property = 80.0f;
            char str_property[1024] = "Hello world";
            char buffer_property[60] = "DEADBEEF";

            auto volatile_properties() {
                return make_protocol_member_list(

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
                    
                    /*,

                    make_protocol_function_kw(
                        *this,
                        &PowernetNamespace::demo_func_with_args,
                        property_name = "demo_func_with_args",
                        function_arguments = std::array<const char *, 1>{
                            "arg_1" }
                    )*/

                );
            };

            struct NvmProperties_t {
                int32_t nv_property = 0;
            } disk0;

            auto non_volatile_properties() {
                return make_protocol_member_list(
                    make_protocol_number_kw(
                        &disk0.nv_property,
                        property_name = "sample_nv_property"
                    )
                );
            }

            event_vector_t event_trigger = {1};

            auto interrupt_properties() {
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
