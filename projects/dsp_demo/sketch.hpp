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

            // Volatile variables
            int32_t adc_sample_number = 0;
            int32_t timer_one_loop_number = 0;

            float adc_samples[2][2][6] = {0.0f};


            // Non volatile variables
            struct DiskStructure_t {
                int32_t nv_property = 0;
            } disk0;

            // Communicable variables
            auto communicable_variables() {
                return make_protocol_member_list(

                    make_protocol_number_kw(
                        &adc_sample_number,
                        property_name = "adc_sample_number",
                        property_is_read_only = true
                    ),

                    make_protocol_number_kw(
                        &timer_one_loop_number,
                        property_name = "timer_one_loop_number",
                        property_is_read_only = true
                    ),

                    make_protocol_number_kw(
                        &adc_samples[0][0][5],
                        property_name = "tim1_adc2",
                        property_is_read_only = true
                    ),
                    make_protocol_number_kw(
                        &adc_samples[0][1][5],
                        property_name = "tim1_adc3",
                        property_is_read_only = true
                    ),
                    make_protocol_number_kw(
                        &adc_samples[1][0][5],
                        property_name = "tim8_adc2",
                        property_is_read_only = true
                    ),
                    make_protocol_number_kw(
                        &adc_samples[1][1][5],
                        property_name = "tim8_adc3",
                        property_is_read_only = true
                    )
                );
            };

            event_vector_t event_trigger = {1};

            auto communicable_interrupts() {
                return make_protocol_member_list(

                    /*make_event_trigger(
                        &event_trigger,
                        property_name = "sample_event"
                    )*/
                );
            };
        };
    #endif

#endif
