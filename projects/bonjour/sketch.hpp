#ifndef __SKETCH_HPP_
    #define __SKETCH_HPP_

    #ifdef INCLUDE_PNTP
    #include "powernet.h"


    // See README.md for information about how to construct 
    // this class

    class PowernetNamespace {

        public:

            int32_t property = 0;

            auto communicable_properties() {
                return make_protocol_member_list(
                    make_protocol_number_kw(
                        &property,
                        property_name = "sample_property"
                    )
                );
            };

            struct DiskStructure_t {
                int32_t nv_property = 0;
            } disk0;

            auto non_communicable_properties() {
                return make_protocol_member_list(
                    make_protocol_number_kw(
                        &disk0.nv_property,
                        property_name = "sample_nv_property"
                    )
                );
            };

            event_vector_t event_trigger = {1};

            auto communicable_interrupts() {
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
