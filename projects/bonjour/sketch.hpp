#ifndef __SKETCH_HPP_
    #define __SKETCH_HPP_

    #ifdef INCLUDE_PNTP
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

    class PntpCommunicable {

        public:

            int32_t property = 0;

            auto customized_volatile_objects() {
                return make_protocol_member_list(
                    make_protocol_number_kw(
                        &property,
                        property_name = "sample_property"
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
            };

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
