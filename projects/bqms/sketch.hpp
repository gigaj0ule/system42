#ifndef __SKETCH_HPP_
    #define __SKETCH_HPP_

    #ifdef INCLUDE_PNTP

    class PowernetNamespace {

        public:

            // BIST I2C Scanner
            //char i2c_addresses[127*6] = {0};

            uint8_t i2c_device_count = 0;

            char i2c_states[5][16] = {
                "SUCCESS",
                "OVERFLOW",
                "NACK ADDR",
                "NACK DATA",
                "MISC ERROR"
            };

            // Current (A)
            float b0_pack_i;
            float b0_adc_temp = 0.0f;
            float b0_i_max;

            // Cell voltages (V)
            float b0_cell_voltage[15] = { 0.0f };
            float b0_cell_voltage_min = 0.0f;
            float b0_cell_voltage_max = 0.0f;
            float b0_cell_voltage_mean = 0.0f;

            // Balancing
            bool b0_cell_is_balancing[15] = { 0 };

            // Pack
            float b0_cell_voltage_total = 0.0f;

            // Cell temps (*C)
            float b0_cell_temperatue[2][15] = {0};
            float b0_cell_temperatue_min = 0.0f;
            float b0_cell_temperatue_max = 0.0f;

            // Limits
            float b0_limit_cell_temperatue_min = 0.0f;
            float b0_limit_cell_temperatue_max = 50.0f;
            float b0_limit_cell_voltage_min = 3.0f;
            float b0_limit_cell_voltage_max = 4.2f;

            // Debugging
            int64_t loop_counter = 0;

            #define TS_A_GND 0
            #define TS_B_GND 1

            // Communicable Properties
            auto volatile_properties() {

                return make_protocol_member_list(

                    // ------------------------------
                    // Cell 0
                    // Is BQ_ADC_CHANNEL 0
                    //
                    make_protocol_object("cell_0",
                        make_protocol_number_kw(
                            &b0_cell_voltage[0],
                            property_name = "v",
                            property_units = "V",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_A_GND][0],
                            property_name = "t_n",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_B_GND][0],
                            property_name = "t_p",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_is_balancing[0],
                            property_name = "bal",
                            property_is_read_only = true
                        )
                    ),

                    // ------------------------------
                    // Cell 1
                    // Is BQ_ADC_CHANNEL 1
                    //
                    make_protocol_object("cell_1",
                        make_protocol_number_kw(
                            &b0_cell_voltage[1],
                            property_name = "v",
                            property_units = "V",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_B_GND][1],
                            property_name = "t_n",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_A_GND][1],
                            property_name = "t_p",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_is_balancing[1],
                            property_name = "bal",
                            property_is_read_only = true
                        )
                    ),

                    // ------------------------------
                    // Cell 2
                    // Is BQ_ADC_CHANNEL 2
                    //
                    make_protocol_object("cell_2",
                        make_protocol_number_kw(
                            &b0_cell_voltage[2],
                            property_name = "v",
                            property_units = "V",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_A_GND][2],
                            property_name = "t_n",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_B_GND][2],
                            property_name = "t_p",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_is_balancing[2],
                            property_name = "bal",
                            property_is_read_only = true
                        )
                    ),

                    // ------------------------------
                    // BQ_ADC_CHANNEL 3 is unused

                    // ------------------------------
                    // Cell 3
                    // Is BQ_ADC_CHANNEL 4
                    //
                    make_protocol_object("cell_3",
                        make_protocol_number_kw(
                            &b0_cell_voltage[4],
                            property_name = "v",
                            property_units = "V",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_B_GND][3],
                            property_name = "t_n",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_A_GND][3],
                            property_name = "t_p",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_is_balancing[3],
                            property_name = "bal",
                            property_is_read_only = true
                        )
                    ),

                    // ------------------------------
                    // Cell 4
                    // Is BQ_ADC_CHANNEL 5
                    //
                    make_protocol_object("cell_4",
                        make_protocol_number_kw(
                            &b0_cell_voltage[5],
                            property_name = "v",
                            property_units = "V",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_A_GND][4],
                            property_name = "t_n",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_B_GND][4],
                            property_name = "t_p",
                            property_units = "*C",
                            property_is_read_only = true
                        )
                        /*,
                        make_protocol_number_kw(
                            &b0_cell_is_balancing[4],
                            property_name = "shunted",
                            property_is_read_only = true
                        )*/
                    ),

                    // ------------------------------
                    // Cell 5
                    // Is BQ_ADC_CHANNEL 6
                    //
                    make_protocol_object("cell_5",
                        make_protocol_number_kw(
                            &b0_cell_voltage[6],
                            property_name = "v",
                            property_units = "V",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_A_GND][5],
                            property_name = "t_n",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_B_GND][5],
                            property_name = "t_p",
                            property_units = "*C",
                            property_is_read_only = true
                        )
                        /*,
                        make_protocol_number_kw(
                            &b0_cell_is_balancing[6],
                            property_name = "shunted",
                            property_is_read_only = true
                        )*/
                    ),

                    // ------------------------------
                    // Cell 6
                    // Is BQ_ADC_CHANNEL 7
                    //
                    make_protocol_object("cell_6",
                        make_protocol_number_kw(
                            &b0_cell_voltage[7],
                            property_name = "v",
                            property_units = "V",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_B_GND][6],
                            property_name = "t_n",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_A_GND][6],
                            property_name = "t_p",
                            property_units = "*C",
                            property_is_read_only = true
                        )
                        /*,
                        make_protocol_number_kw(
                            &b0_cell_is_balancing[7],
                            property_name = "shunted",
                            property_is_read_only = true
                        )*/
                    ),

                    // ------------------------------
                    // BQ_ADC_CHANNEL 8 is unused

                    // ------------------------------
                    // Cell 7
                    // Is BQ_ADC_CHANNEL 9
                    //
                    make_protocol_object("cell_7",
                        make_protocol_number_kw(
                            &b0_cell_voltage[9],
                            property_name = "v",
                            property_units = "V",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_B_GND][7],
                            property_name = "t_n",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_A_GND][7],
                            property_name = "t_p",
                            property_units = "*C",
                            property_is_read_only = true
                        )
                        /*,
                        make_protocol_number_kw(
                            &b0_cell_is_balancing[9],
                            property_name = "shunted",
                            property_is_read_only = true
                        )*/
                    ),

                    // ------------------------------
                    // Cell 8
                    // Is BQ_ADC_CHANNEL 10
                    //
                    make_protocol_object("cell_8",
                        make_protocol_number_kw(
                            &b0_cell_voltage[10],
                            property_name = "v",
                            property_units = "V",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_A_GND][8],
                            property_name = "t_n",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_B_GND][8],
                            property_name = "t_p",
                            property_units = "*C",
                            property_is_read_only = true
                        )
                        /*,
                        make_protocol_number_kw(
                            &b0_cell_is_balancing[10],
                            property_name = "shunted",
                            property_is_read_only = true
                        )*/
                    ),

                    // ------------------------------
                    // Cell 9
                    // Is BQ_ADC_CHANNEL 11
                    //
                    make_protocol_object("cell_9",
                        make_protocol_number_kw(
                            &b0_cell_voltage[11],
                            property_name = "v",
                            property_units = "V",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_B_GND][9],
                            property_name = "t_n",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_A_GND][9],
                            property_name = "t_p",
                            property_units = "*C",
                            property_is_read_only = true
                        )
                        /*,
                        make_protocol_number_kw(
                            &b0_cell_is_balancing[11],
                            property_name = "shunted",
                            property_is_read_only = true
                        )*/
                    ),

                    // ------------------------------
                    // Cell 10
                    // Is BQ_ADC_CHANNEL 12
                    //
                    make_protocol_object("cell_10",
                        make_protocol_number_kw(
                            &b0_cell_voltage[12],
                            property_name = "v",
                            property_units = "V",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_A_GND][10],
                            property_name = "t_n",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_B_GND][10],
                            property_name = "t_p",
                            property_units = "*C",
                            property_is_read_only = true
                        )
                        /*,
                        make_protocol_number_kw(
                            &b0_cell_is_balancing[12],
                            property_name = "shunted",
                            property_is_read_only = true
                        )*/
                    ),

                    // ------------------------------
                    // BQ_ADC_CHANNEL 13 is unused

                    // ------------------------------
                    // Cell 11
                    // Is BQ_ADC_CHANNEL 14
                    //
                    make_protocol_object("cell_11",
                        make_protocol_number_kw(
                            &b0_cell_voltage[14],
                            property_name = "v",
                            property_units = "V",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_B_GND][11],
                            property_name = "t_n",
                            property_units = "*C",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_cell_temperatue[TS_A_GND][11],
                            property_name = "t_p",
                            property_units = "*C",
                            property_is_read_only = true
                        )
                        /*,
                        make_protocol_number_kw(
                            &b0_cell_is_balancing[14],
                            property_name = "shunted",
                            property_is_read_only = true
                        )*/
                    ),

                    // ------------------------------
                    // Total Battery Pack
                    //
                    make_protocol_object("battery",
                        make_protocol_number_kw(
                            &b0_cell_voltage_total,
                            property_name = "voltage",
                            property_units = "V",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_pack_i,
                            property_name = "current",
                            property_units = "A",
                            property_is_read_only = true
                        ),
                        make_protocol_number_kw(
                            &b0_adc_temp,
                            property_name = "adc_temp",
                            property_units = "*C",
                            property_is_read_only = true
                        )
                    ),

                    // ------------------------------
                    // Debug
                    //
                    make_protocol_object("debug",

                        make_protocol_number_kw(
                            &loop_counter,
                            property_name = "loop_counter",
                            property_is_read_only = true
                        ),

                        make_protocol_number_kw(
                            &i2c_device_count,
                            property_name = "i2c_device_count",
                            property_is_read_only = true
                        )
                        /*,
                        make_protocol_string_kw(
                            &i2c_addresses,
                            property_name = "i2c_addresses",
                            property_length = sizeof(i2c_addresses),
                            property_is_read_only = true
                        )*/
                    )
                );
            };

            struct NvmProperties_t {
                int32_t nv_property = 0;
            } disk0;

            auto non_volatile_properties() {
                return make_protocol_member_list();
            }

            //event_vector_t scan_complete_event = {1};

            auto interrupt_properties() {
                return make_protocol_member_list(

                    /*make_event_trigger(
                        &scan_complete_event,
                        property_name = "scan_complete"
                    )*/
                );
            };
    };

    #endif

#endif
