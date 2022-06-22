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
            char device_make[16] = "i2c_scan";
            char device_model[16]  = "stm42";

            char i2c_addresses[127*6] = {0};

            uint8_t i2c_device_count = 0;

            int32_t v_cells[14] = {0};
            float v_cells_f[14] = {0};

            float v_diode_bias = 0;
            float v_diode_bias_f = 0;


            char i2c_states[5][16] = {
                "SUCCESS",
                "OVERFLOW",
                "NACK ADDR",
                "NACK DATA",
                "MISC ERROR"
            };

            auto customized_volatile_objects() {
                return make_protocol_member_list(

                    make_protocol_object("v_int",
                        make_protocol_number_kw(
                            &v_cells[0],
                            property_name = "v_c1",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells[1],
                            property_name = "v_c2",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells[2],
                            property_name = "v_c3",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells[3],
                            property_name = "v_c4",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells[4],
                            property_name = "v_c5",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells[5],
                            property_name = "v_c6",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells[6],
                            property_name = "v_c7",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells[7],
                            property_name = "v_c8",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells[8],
                            property_name = "v_c9",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells[9],
                            property_name = "v_c10",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells[10],
                            property_name = "v_c11",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells[11],
                            property_name = "v_c12",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells[12],
                            property_name = "v_sample",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells[13],
                            property_name = "v_actual",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_diode_bias,
                            property_name = "v_diode_bias",
                            property_is_read_only = true
                        )
                    ),


                    make_protocol_object("v_float",
                        make_protocol_number_kw(
                            &v_cells_f[0],
                            property_name = "v_c1",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells_f[1],
                            property_name = "v_c2",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells_f[2],
                            property_name = "v_c3",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells_f[3],
                            property_name = "v_c4",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells_f[4],
                            property_name = "v_c5",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells_f[5],
                            property_name = "v_c6",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells_f[6],
                            property_name = "v_c7",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells_f[7],
                            property_name = "v_c8",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells_f[8],
                            property_name = "v_c9",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells_f[9],
                            property_name = "v_c10",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells_f[10],
                            property_name = "v_c11",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells_f[11],
                            property_name = "v_c12",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells_f[12],
                            property_name = "v_sample",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_cells_f[13],
                            property_name = "v_actual",
                            property_is_read_only = false
                        ),
                        make_protocol_number_kw(
                            &v_diode_bias_f,
                            property_name = "v_diode_bias",
                            property_is_read_only = true
                        )
                    )
                );

            };

            struct NvmProperties_t {
                int32_t nv_property = 0;
            } non_volatile_properties;

            auto customized_nonvolatile_objects() {
                return make_protocol_member_list();
            }

            event_vector_t scan_complete_event = {1};

            auto customized_interrupts() {
                return make_protocol_member_list(

                    make_event_trigger(
                        &scan_complete_event,
                        property_name = "scan_complete"
                    )
                );
            };
        };
    #endif

#endif
