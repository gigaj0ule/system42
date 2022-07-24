#ifndef __SENSORLESS_ESTIMATOR_HPP
#define __SENSORLESS_ESTIMATOR_HPP

// The test frequency for the HFI, as subtracted from PWM_HZ
// It is required to use wierd numbers as it reduces beat harmonics

// Note, if you change this, you will need to check all the
// filters. Otherwise you will end up with ambiguous behavior.
// For the love of all that is holy don't set it lower than the
// first filter pass band or your ears will be ringing.

// You should set it to a non-integer or you'll likely encounter
// beat frequency problems.

// You should set it low enough that the self inductance of the motor
// does not ruin your signal, but high enough that the signal does not
// interfere with the fundamental frequency of your control loop.

#define HFI_TEST_FREQUENCY 1046.5f //1760 //880 //440

// This sets the resolution of the compensation map, don't set it
// to low or the system will destabilize.
#define HFI_COMPENSATION_MAP_NUMBER_OF_ANGLES  180
#define HFI_COMPENSATION_MAP_NUMBER_OF_CURRENTS 16

//#define HFI_DEBUGING_INSPECT_IC_RESPONSE
//#define HFI_DEBUGGING_INSPECT_ESTIMATOR
//#define HFI_DEBUGGING_INSPECT_ESTIMATOR_DC_CAL_RESULTS
//#define HFI_DEBUGGING_INSPECT_DRIVE_CURRENT


class SensorlessEstimator {
public:
    enum Error_t : int32_t {
        ERROR_NONE = 0,
        ERROR_UNSTABLE_GAIN = 0x01,
        ERROR_NAN_FLUX_STATE = 0x02
    };

    Error_t error_ = ERROR_NONE;

    struct Config_t {
        float observer_gain     = 1000.0f;      // [rad/s]
        float pll_bandwidth     = 1000.0f;      // [rad/s]
        float pm_flux_linkage   = 1.58e-3f;     // [V / (rad/s)]  { 5.51328895422 / (<pole pairs> * <rpm/v>) }

        bool hfi_enabled = true;
        bool hfi_calibrated = false;

        float test_signal_voltage  = 0.0f;          // [V]
        float test_signal_desired_snr = 10.0f;      // [V/V]
        float hfi_max_calibration_current = 10.0f;  // [A]

        float hfi_I_real_nonlinearity_table[HFI_COMPENSATION_MAP_NUMBER_OF_ANGLES][HFI_COMPENSATION_MAP_NUMBER_OF_CURRENTS] = {0.0f};
        //float hfi_I_imag_nonlinearity_table[HFI_COMPENSATION_MAP_NUMBER_OF_ANGLES][HFI_COMPENSATION_MAP_NUMBER_OF_CURRENTS] = {0.0f};

        float hfi_I_real_norm_[2][HFI_COMPENSATION_MAP_NUMBER_OF_CURRENTS] = {0.0f};
        //float hfi_I_imag_norm_[2][HFI_COMPENSATION_MAP_NUMBER_OF_CURRENTS] = {0.0f};

        // Used with our nonlinearity table, not to be set by user
        float calibration_current_inv_increment = 0.0f;

        // Starts to fail around 500 rad/s
        float hfi_max_velocity      = 400.0f;       //[rad/s]
        float hfi_vel_hysteresis    = 0.25f;

        // What this really needs to be is a function of motor velocity
        float hfi_torque_gain   = 100.0f;           // Hz
        float hfi_torque_decay  = 1.0f;             // Hz
        // end todo

        float hfi_quiescent_current = 1.0f;         // A

    };

    explicit SensorlessEstimator(Config_t &config);

    bool update();
    bool reset_state();

    Axis* axis_ = nullptr; // set by Axis constructor
    Config_t &config_;

    float phase_ = 0.0f;                            // [rad]
    float pll_pos_ = 0.0f;                          // [rad]

    float vel_estimate_ = 0.0f;                     // [rad/s]

    // low pass filtered vel_estimate
    float vel_estimate_filter_constant_ = 2.0f * 1.0f / current_meas_hz;
    float vel_estimate_filtered_ = 0.0f;            // [rad/s]

    // float pll_kp_ = 0.0f;                        // [rad/s / rad]
    // float pll_ki_ = 0.0f;                        // [(rad/s^2) / rad]
    float flux_state_[2] = {0.0f, 0.0f};            // [Vs]
    float V_alpha_beta_memory_[2] = {0.0f, 0.0f};   // [V]


    // HFI estimator -------------------------------------------
    bool measure_hfi_intrinsics();
    void sample_hfi(float * I_phase, float * I_alpha, float * I_beta);
    void update_hfi(float * I_d_measured);
    void generate_new_hfi_modulations(float * I_phase, float * V_to_mod, float * hfi_modulation_index, float * hfi_alphas, float * hfi_betas);

    // Bandwidth of the filter used for increasing current in the
    // direct mode current control loop. Should be kept low on motors with
    // lots of secondary saliency. (maybe this should go in controller)
    float hfi_misalignment_filter_bandwidth_ = 10.0f;

    float hfi_delta_phase_measurement_linearized_ = 0;

    // Phase which is incremented for creating the high frequency injection signal
    float hfi_carrier_phase_ = 0.0f;

    // Injection angle of the test signal, functionally equivalent to
    // phase estimate in normal operation
    float hfi_test_signal_injection_angle_ = 0.0f;
    bool hfi_closed_feedback_loop_ = true;

    // Is HFI "on" or not?
    bool hfi_engaged = false;

    // This array is for referencing the correct HFI phase angles which were sampled according to
    // the state machine in low_level_fast.cpp
    const uint8_t hfi_loop_n_minus_2_references_supposedly_correct_[3] = {1, 3, 5};

    // These arrays store values of phases and sin/cos which were computed on FOC loop N-1 and N-2
    float hfi_stashed_injection_I_phases_[2] = {0.0f};
    float hfi_stashed_injection_angles_[2] = {0.0f};    // Equivalent to previous array + M_PI
    float hfi_stashed_d_currents_[2] = {0.0f};

    float hfi_carrier_phases_[12] = {0.0f};

    float hfi_carrier_sin_p_[12] = {0.0f};
    float hfi_carrier_sin_n_[12] = {0.0f};

    float hfi_carrier_cos_p_[12] = {0.0f};
    float hfi_carrier_cos_n_[12] = {0.0f};

    float hfi_injection_angles_[12] = {0.0f};
    float hfi_injection_angle_sin_[12] = {0.0f};
    float hfi_injection_angle_cos_[12] = {0.0f};

    float hfi_phases_b_[12] = {0.0f};
    float hfi_sin_b_[12] = {0.0f};
    float hfi_cos_b_[12] = {0.0f};

    float hfi_I_p_real_DC_filtered_average_ = 0;

    // This keeps track of where we are in the 12 phases (array partition)
    bool hfi_buffer_ping_pong_ = 0;

    // Filter states for subtracting DC from HFI signals
    float hfi_I_alpha_filter_state_[3] = {0};
    float hfi_I_beta_filter_state_[3]  = {0};

    float hfi_I_p_real_DC_filter_state_[3][3] = {0};
    float hfi_I_p_imag_DC_filter_state_[3][3] = {0};

    float hfi_I_n_real_filter_state_[3] = {0};
    float hfi_I_n_imag_filter_state_[3] = {0};

    float hfi_I_n_real_DC_filter_state_[3] = {0};
    float hfi_I_n_imag_DC_filter_state_[3] = {0};

    float hfi_In_d_salient_average_filtered_ = 0.0f;
    float hfi_In_q_salient_average_filtered_ = 0.0f;

    // Stuff for DFT (compensation attempt)
    bool hfi_run_calibration_integrator_ = 0;

    struct DFTintegrals_t {
        bool enable_filter = 0;
        float filter_state[3] = {0};
        float I_real_min = 0;
        float I_real_max = 0;
        float I_imag_max = 0;
        float I_imag_min = 0;
        uint32_t samples = 0;
    };

    DFTintegrals_t hfi_calibration_;

    // Estimates from HFI observer
    float hfi_delta_phase_estimate_ = 0;
    float hfi_delta_vel_estimate_   = 0;
    float hfi_phase_accel_estimate_ = 0;
    float hfi_d_current_estimate_   = 0;

    // Debugging
    float debug_[10] = {0};

    float hfi_cleaned_I_alpha_ = 0;
    float hfi_cleaned_I_beta_ = 0;

    float hfi_I_differential_real_average_last_iteration_ = 0;
    float hfi_I_p_imag_filtered_last_iteration_ = 0;
    float hfi_I_n_imag_filtered_last_iteration_ = 0;

    float hfi_I_differential_real_average_bias_removed_ = 0;
    float hfi_I_p_imag_filtered_bias_removed_ = 0;
    float hfi_I_n_imag_filtered_bias_removed_ = 0;

    float hfi_I_differential_real_average_ = 0;
    float hfi_I_differential_imag_average_ = 0;

    float hfi_delta_phase_measurement_linearized_last_ = 0;

    float hfi_I_differential_real_ = 0;

    float hfi_I_real_nonlinearity_table_filter_state_[3] = {0};
    float hfi_I_imag_nonlinearity_table_filter_state_[3] = {0};

    float hfi_q_current_filter_ = 0;
    float hfi_d_current_filter_ = 0;

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_number_kw(
                &error_,
                property_name = "error"
            ),
            make_protocol_number_kw(
                &phase_,
                property_name = "phase",
                property_units = "rad"
            ),
            make_protocol_number_kw(
                &pll_pos_,
                property_name = "pll_pos",
                property_units = "rad"
            ),
            make_protocol_number_kw(
                &vel_estimate_filtered_,
                property_name = "vel_estimate_filtered",
                property_units = "rad/s"
            ),
            // make_protocol_number_kw("pll_kp", &pll_kp_),
            // make_protocol_number_kw("pll_ki", &pll_ki_),

            /*
            make_protocol_object("hfi_debug",
                make_protocol_number_kw("debug_0", &debug_[0]),
                make_protocol_number_kw("debug_1", &debug_[1]),
                make_protocol_number_kw("debug_2", &debug_[2]),
                make_protocol_number_kw("debug_3", &debug_[3]),
                make_protocol_number_kw("debug_4", &debug_[4]),
                make_protocol_number_kw("debug_5", &debug_[5]),
                make_protocol_number_kw("debug_6", &debug_[6]),
                make_protocol_number_kw("debug_7", &debug_[7]),
                make_protocol_number_kw("debug_8", &debug_[8]),
                make_protocol_number_kw("debug_8", &debug_[9])
            ),
            */

            make_protocol_object("config",

                make_protocol_number_kw(
                    &config_.observer_gain,
                    property_name = "observer_gain",
                    property_units = "rad/s"
                ),
                make_protocol_number_kw(
                    &config_.pll_bandwidth,
                    property_name = "pll_bandwidth",
                    property_units = "rad/s"
                ),
                make_protocol_number_kw(
                    &config_.pm_flux_linkage,
                    property_name = "pm_flux_linkage"
                ),

                //make_protocol_object("hfi",
                    make_protocol_number_kw(
                        &config_.hfi_enabled,
                        property_name = "hfi_enabled"
                    ),
                    make_protocol_number_kw(
                        &config_.hfi_calibrated,
                        property_name = "hfi_calibrated"
                    ),
                    make_protocol_number_kw(
                        &config_.test_signal_desired_snr,
                        property_name = "hfi_signal_snr_target"
                    ),
                    make_protocol_number_kw(
                        &config_.test_signal_voltage,
                        property_name = "hfi_signal_voltage",
                        property_units = "V"
                    ),
                    make_protocol_number_kw(
                        &config_.hfi_quiescent_current,
                        property_name = "hfi_quiescent_current",
                        property_units = "A"
                    ),
                    make_protocol_number_kw(
                        &config_.hfi_max_calibration_current,
                        property_name = "hfi_max_calibration_current",
                        property_units = "A"
                    ),
                    make_protocol_number_kw(
                        &config_.hfi_max_velocity,
                        property_name = "hfi_max_velocity"
                    ),
                    make_protocol_number_kw(
                        &config_.hfi_vel_hysteresis,
                        property_name = "hfi_velocity_hysteresis"
                    ),
                    make_protocol_number_kw(
                        &config_.hfi_torque_gain,
                        property_name = "hfi_torque_gain"
                    ),
                    make_protocol_number_kw(
                        &config_.hfi_torque_decay,
                        property_name = "hfi_torque_decay"
                    )
                //)
            )
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(SensorlessEstimator::Error_t)

#endif /* __SENSORLESS_ESTIMATOR_HPP */