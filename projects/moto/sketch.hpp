#ifndef __SKETCH_HPP_
    #define __SKETCH_HPP_

    #include "protocol.hpp"
    #include "communication.h"

    class PowernetNamespace {

        public:
            char device_make[16] = "moto";
            char device_model[16] = "v100";

            auto communicable_variables() {
                return make_protocol_member_list();
            };

            struct DiskStructure_t {
                uint32_t nv_property = 0;
            } non_communicable_variables;

            auto noncommunicable_variables() {
                return make_protocol_member_list();
            }

            auto communicable_interrupts() {
                return make_protocol_member_list();
            };
    };

#endif