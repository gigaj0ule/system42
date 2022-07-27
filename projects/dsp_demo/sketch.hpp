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

            // Non volatile variables
            struct DiskStructure_t {
                int32_t nv_property = 0;
            } disk0;

            // Communicable variables
            auto communicable_variables() {
                return make_protocol_member_list(

                    make_protocol_number_kw(
                        &adc_sample_number,
                        property_name = "adc_sample_number"
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
