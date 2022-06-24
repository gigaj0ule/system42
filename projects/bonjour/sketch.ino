//  Illustrates how to register a PNTP Server.
//

// Headers
#include "sketch_only_statics.hpp"

#include "powernet.h"

// See README.md for information about how to construct 
// this class

class PowernetNamespace {

    public:

        // Volatile Properties
        //
        int sample_v_property = 0;

        auto volatile_properties() {
            return make_protocol_member_list(
                make_protocol_number_kw(
                    &sample_v_property,
                    property_name = "sample_v_property"
                )
            );
        };

        // Nonvolatile Properties
        //
        struct NvmProperties_t {
            int32_t sample_nonvolatile_property = 0;
        } disk0;

        auto nonvolatile_properties() {
            return make_protocol_member_list(
                make_protocol_number_kw(
                    &disk0.sample_nonvolatile_property,
                    property_name = "sample_nonvolatile_property",
                    property_is_non_volatile = true
                )
            );
        }

        // Interrupt properties
        //
        event_vector_t sample_interrupt = {1};

        auto interrupt_properties() {
            return make_protocol_member_list(

                make_event_trigger(
                    &sample_interrupt,
                    property_name = "sample_interrupt"
                )
            );
        };
};

PowernetNamespace pntp;


// Runs Once
void setup() {
    
    pntp_begin("stm32 bonjour");
};


// Runs Forever
void loop() {

	pntp_listen();
};