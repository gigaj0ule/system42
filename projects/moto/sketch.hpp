#ifndef __SKETCH_HPP_
    #define __SKETCH_HPP_

    #include "protocol.hpp"
    #include "communication.h"

    class PowernetNamespace {

        public:
            char device_make[16] = "moto";
            char device_model[16] = "v100";

            auto volatile_properties() {
                return make_protocol_member_list();
            };

            struct NvmProperties_t {
                uint32_t nv_property = 0;
            } non_volatile_properties;

            auto nonvolatile_properties() {
                return make_protocol_member_list();
            }

            auto interrupt_properties() {
                return make_protocol_member_list();
            };
    };

#endif