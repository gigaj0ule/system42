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

            // Make sure these line up with the LED types in sketch.ino!
            char led_types[35][16] = {

                // 0
                "LPD6803", 
                "LPD8806",
                "WS2801",
                "WS2803",
                "SM16716",
                "P9813",
                "APA102",
                "SK9822",
                "DOTSTAR",
                "SM16703",

                // 10
                "TM1829",
                "TM1812",
                "TM1809",
                "TM1804",
                "TM1803",
                "UCS1903",
                "UCS1903B",
                "UCS1904",
                "UCS2903",
                "WS2812",

                // 20              
                "WS2852",
                "WS2812B",
                "GS1903",
                "SK6812",
                "APA106",
                "PL9823",
                "WS2811",
                "WS2813",
                "APA104",
                "WS2811_400",

                // 30
                "GE8822",
                "GW6205",
                "GW6205_400",
                "LPD1886",
                "LPD1886_8BIT"
            };

            auto volatile_properties() {
                return make_protocol_member_list(

                    /*make_protocol_selection_kw(
                        &led_type, 
                        property_name = "led_type", 
                        property_option_strings = led_types, 
                        property_option_count = 34
                    ),*/

                    make_protocol_buffer_kw(
                        &led_banks[0], 
                        property_name = "led_data", 
                        property_length = NUMBER_OF_BYTES_PER_LED_BANK
                    )
                );
            }

            struct NvmProperties_t {
                int32_t sample_property = 0;
            } disk0;

            auto non_volatile_properties() {
                return make_protocol_member_list();
            }

            auto interrupt_properties() {
                return make_protocol_member_list();
            };
    };

    #endif

#endif