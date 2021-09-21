#ifndef __SKETCH_HPP_
    #define __SKETCH_HPP_

    #include "protocol.hpp"
    #include "communication.h"

    class communication {

        public:
            char device_make[16] = "inverter";
            char device_model[16] = "v100";

            auto customized_volatile_objects() {
                return make_protocol_member_list();
            };

            struct NvmProperties_t {
                uint32_t nv_property = 0;
            } non_volatile_properties;

            auto customized_nonvolatile_objects() {
                return make_protocol_member_list();
            }

            auto customized_event_triggers() {
                return make_protocol_member_list();
            };
    };

#endif