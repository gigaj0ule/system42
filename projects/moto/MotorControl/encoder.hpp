#ifndef __ENCODER_HPP
#define __ENCODER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

class Encoder {
public:
    enum Error_t: int32_t {
        ERROR_NONE = 0,
        ERROR_UNSTABLE_GAIN = 0x01,
        ERROR_CPR_OUT_OF_RANGE = 0x02,
        ERROR_NO_RESPONSE = 0x04,
        ERROR_UNSUPPORTED_ENCODER_MODE = 0x08,
        ERROR_ILLEGAL_HALL_STATE = 0x10,
        ERROR_INDEX_NOT_FOUND_YET = 0x20,
    };

    enum Mode_t: int32_t {
        MODE_INCREMENTAL,
        MODE_HALL,
        MODE_SINCOS
    };

    // Make sure these line up with the enumerated types!
    char encoder_modes_[3][16] = {
        // 0
        "INCREMENTAL",
        "HALL",
        "SINCOS"
    };

    struct Config_t {
        #ifdef HOVERBOARD_SETTINGS
        Encoder::Mode_t mode = Encoder::MODE_HALL;
        #else
        Encoder::Mode_t mode = Encoder::MODE_INCREMENTAL;
        #endif
        bool use_index = false;
        bool pre_calibrated = false; // If true, this means the offset stored in
                                    // configuration is valid and does not need
                                    // be determined by run_offset_calibration.
                                    // In this case the encoder will enter ready
                                    // state as soon as the index is found.
        bool zero_count_on_find_idx = true;
        #ifdef HOVERBOARD_SETTINGS
        int32_t cpr = 90;
        #else
        int32_t cpr = (2048 * 4);   // Default resolution of CUI-AMT102 encoder,
        #endif
        int32_t offset = 0;        // Offset between encoder count and rotor electrical phase
        float offset_float = 0.0f; // Sub-count phase alignment offset
        bool enable_phase_interpolation = true; // Use velocity to interpolate inside the count state
        float calib_range = 0.05f; // Accuracy required to pass encoder cpr check
        // Set min vel for phase interpolation
        int32_t min_phase_interpolation_vel = 100; // [rad/s electrical]
        #ifdef HOVERBOARD_SETTINGS
        float bandwidth = 100.0f;
        #else
        float bandwidth = 1000.0f;
        #endif
        bool find_idx_on_lockin_only = false; // Only be sensitive during lockin scan constant vel state
        bool idx_search_unidirectional = false; // Only allow index search in known direction

        bool ignore_illegal_hall_state = true; // dont error on bad states like 000 or 111
    };

    Encoder(const EncoderHardwareConfig_t& hw_config,
                     Config_t& config);

    void setup();
    void set_error(Error_t error);
    bool do_checks();

    void enc_index_cb();
    void set_idx_subscribe(bool override_enable = false);
    //void update_pll_gains();
    void check_pre_calibrated();

    void set_linear_count(int32_t count);
    void set_circular_count(int32_t count, bool update_offset);
    bool calib_enc_offset(float voltage_magnitude);
    bool scan_for_enc_idx(float omega, float voltage_magnitude);

    bool run_index_search();
    bool run_direction_find();
    bool run_offset_calibration();
    void sample_now();
    bool update();



    const EncoderHardwareConfig_t& hw_config_;
    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    Error_t error_ = ERROR_NONE;
    bool index_found_ = false;
    bool is_ready_ = false;
    int32_t shadow_count_ = 0;
    int32_t count_in_cpr_ = 0;
    float interpolation_ = 0.0f;
    float phase_ = 0.0f;    // [count]
    float pos_estimate_ = 0.0f;  // [count]
    float pos_cpr_ = 0.0f;  // [count]
    float vel_estimate_ = 0.0f;  // [count/s]
    float pll_kp_ = 0.0f;   // [count/s / count]
    float pll_ki_ = 0.0f;   // [(count/s^2) / count]

    int16_t tim_cnt_sample_ = 0; //
    // Updated by low_level pwm_adc_cb
    uint8_t hall_state_ = 0x0; // bit[0] = HallA, .., bit[2] = HallC
    float sincos_sample_s_ = 0.0f;
    float sincos_sample_c_ = 0.0f;

    // Communication protocol definitions
    auto make_protocol_definitions() {

        return make_protocol_member_list(

            make_protocol_number_kw(
                &error_,
                property_name = "error"
            ),
            make_protocol_number_kw(
                &is_ready_,
                property_name = "is_ready"
            ),
            make_protocol_number_kw(
                const_cast<bool*>(&index_found_),
                property_name = "index_found"
            ),
            make_protocol_number_kw(
                &shadow_count_,
                property_name = "shadow_count"
            ),
            make_protocol_number_kw(
                &count_in_cpr_,
                property_name = "count_in_cpr"
            ),
            make_protocol_number_kw(
                &interpolation_,
                property_name = "interpolation"
            ),
            make_protocol_number_kw(
                &phase_,
                property_name = "phase"
            ),
            make_protocol_number_kw(
                &pos_estimate_,
                property_name = "pos_estimate"
            ),
            make_protocol_number_kw(
                &pos_cpr_,
                property_name = "pos_cpr"
            ),
            make_protocol_number_kw(
                &hall_state_,
                property_name = "hall_state"
            ),
            make_protocol_number_kw(
                &vel_estimate_,
                property_name = "vel_estimate"
            ),

            // make_protocol_number_kw("pll_kp", &pll_kp_),
            // make_protocol_number_kw("pll_ki", &pll_ki_),

            make_protocol_object("config",

                make_protocol_selection_kw(
                    (int32_t *) &config_.mode,
                    property_name = "mode",
                    property_option_strings = encoder_modes_,
                    property_option_count = 3
                ),
                make_protocol_number_kw(
                    &config_.use_index,
                    property_name = "use_index"
                    //after_written_callback = [](void* ctx) { static_cast<Encoder*>(ctx)->set_idx_subscribe(); }
                ),
                make_protocol_number_kw(
                    &config_.find_idx_on_lockin_only,
                    property_name = "find_idx_on_lockin_only"
                    //after_written_callback = [](void* ctx) { static_cast<Encoder*>(ctx)->set_idx_subscribe(); }
                ),
                /*
                make_protocol_function_kw(
                    *this,
                    &Encoder::set_idx_subscribe,
                    property_name = "update_index_settings"
                ),*/
                make_protocol_number_kw(
                    const_cast<bool *>(&config_.pre_calibrated),
                    property_name = "pre_calibrated"
                    //[](void* ctx) { static_cast<Encoder*>(ctx)->check_pre_calibrated(); }, this),
                ),
                make_protocol_function_kw(
                    *this,
                    &Encoder::check_pre_calibrated,
                    property_name = "check_pre_calibrated"
                ),         
                make_protocol_number_kw(
                    &config_.zero_count_on_find_idx,
                    property_name = "zero_count_on_find_idx"
                ),
                make_protocol_number_kw(
                    &config_.cpr,
                    property_name = "cpr"
                ),
                make_protocol_number_kw(
                    &config_.offset,
                    property_name = "offset"
                ),
                make_protocol_number_kw(
                    &config_.offset_float,
                    property_name = "offset_float"
                ),
                make_protocol_number_kw(
                    &config_.enable_phase_interpolation,
                    property_name = "enable_phase_interpolation"
                ),
                make_protocol_number_kw(
                    &config_.min_phase_interpolation_vel,
                    property_name = "min_phase_interpolation_vel"
                ),
                make_protocol_number_kw(
                    &config_.bandwidth, 
                    property_name = "bandwidth" 
                    //after_written_callback = [](void* ctx) { static_cast<Encoder*>(ctx)->update_pll_gains(); }, 
                ),  
                make_protocol_number_kw(
                    &config_.calib_range,
                    property_name = "calib_range"
                ),
                make_protocol_number_kw(
                    &config_.idx_search_unidirectional,
                    property_name = "idx_search_unidirectional"
                ),
                make_protocol_number_kw(
                    &config_.ignore_illegal_hall_state,
                    property_name = "ignore_illegal_hall_state"
                ),
                make_protocol_function_kw(
                    *this, 
                    &Encoder::set_linear_count, 
                    property_name = "set_linear_count",
                    function_arguments = std::array<const char *, 1>{"count"}
                )
            )
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(Encoder::Error_t)

#endif // __ENCODER_HPP
