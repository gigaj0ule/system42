#ifndef __SKETCH_HPP_
    #define __SKETCH_HPP_

    #ifdef INCLUDE_PNTP
    #include "powernet.h"

    // See README.md for information about how to construct 
    // this class

    #define NUMBER_OF_LEDS_PER_BANK 128
    #define NUMBER_OF_LED_BANKS 1
    #define NUMBER_OF_BYTES_PER_LED_BANK 4 * NUMBER_OF_LEDS_PER_BANK
    #define NUMBER_OF_LEDS_TOTAL NUMBER_OF_LED_BANKS * NUMBER_OF_LEDS_PER_BANK

    class PowernetNamespace {

        public:

            char led_banks[NUMBER_OF_LED_BANKS][NUMBER_OF_BYTES_PER_LED_BANK] = {0};

            auto communicable_variables() {
                return make_protocol_member_list(

                    make_protocol_buffer_kw(
                        &led_banks[0], 
                        property_name = "led_data", 
                        property_length = NUMBER_OF_BYTES_PER_LED_BANK
                    )
                );
            }

            struct DiskStructure_t {
                int32_t sample_property = 0;
            } disk0;

            auto communicable_interrupts() {
                return make_protocol_member_list();
            };
    };

    #endif

#endif