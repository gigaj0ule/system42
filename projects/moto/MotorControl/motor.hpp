#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

#include "drv8301.hpp"

class Motor {
public:
    enum Error_t : int32_t {
        ERROR_NONE = 0,
        ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 0x0001,
        ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 0x0002,
        ERROR_ADC_FAILED = 0x0004,
        ERROR_DRV_FAULT = 0x0008,
        ERROR_CONTROL_DEADLINE_MISSED = 0x0010,
        ERROR_NOT_IMPLEMENTED_MOTOR_TYPE = 0x0020,
        ERROR_BRAKE_CURRENT_OUT_OF_RANGE = 0x0040,
        ERROR_MODULATION_MAGNITUDE = 0x0080,
        ERROR_BRAKE_DEADTIME_VIOLATION = 0x0100,
        ERROR_UNEXPECTED_TIMER_CALLBACK = 0x0200,
        ERROR_CURRENT_SENSE_SATURATION = 0x0400,
        ERROR_INVERTER_OVER_TEMP = 0x0800,
        ERROR_PM_FLUX_LINKAGE_OUT_OF_RANGE = 0x1000,
        ERROR_UNEXPECTED_ADC_CALLBACK = 0x2000
    };

    enum MotorType_t : int32_t {
        MOTOR_TYPE_HIGH_CURRENT = 0,
        // MOTOR_TYPE_LOW_CURRENT = 1, //Not yet implemented
        MOTOR_TYPE_GIMBAL = 2
    };

    // Make sure these line up with the enumerated types!
    char motor_types_[3][16] = {
        // 0
        "HIGH_CURRENT",
        "unimplemented",
        "GIMBAL"
    };

    enum TimingsType_t : int32_t  {
        VOLTAGE_TIMINGS = 0,
        MODULATION_TIMINGS = 1
    };

    enum AplhaBetaType_t  {
        TYPE_ALPHA = 0,
        TYPE_BETA = 1
    };

    struct Iph_BC_t {
        float phB;
        float phC;
    };

    struct CurrentControl_t {
        float p_gain; // [V/A]
        float i_gain; // [V/As]
        float p_gain_max; // [V/A]
        float i_gain_max; // [V/As]
        float v_current_control_integral_d; // [V]
        float v_current_control_integral_q; // [V]
        float Ibus; // DC bus current [A]
        // Voltage applied at end of cycle:
        float final_v_alpha; // [V]
        float final_v_beta; // [V]
        float Iq_setpoint; // [A]
        float Iq_measured; // [A]
        float Id_setpoint; // [A]
        float Id_measured; // [A]
        float I_measured_report_filter_k;
        float max_allowed_current; // [A]
        float overcurrent_trip_level; // [A]
    };

    float saved_steady_state_v_current_control_integral_d_;
    float saved_steady_state_v_current_control_integral_q_;

    // NOTE: for gimbal motors, all units of A are instead V.
    // example: vel_gain is [V/(count/s)] instead of [A/(count/s)]
    // example: current_lim and calibration_current will instead determine the maximum voltage applied to the motor.
    struct Config_t {
        bool pre_calibrated = true; // can be set to true to indicate that all values here are valid

        #ifdef HOVERBOARD_SETTINGS
        int32_t pole_pairs = 15;
        #else
        int32_t pole_pairs = 7;
        #endif

        float calibration_current = 5.0f;    // [A]
        float resistance_calib_max_voltage = 5.0f; // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.
        float phase_inductance = 0.01f;        // to be set by measure_phase_inductance
        float phase_resistance = 0.01f;        // to be set by measure_phase_resistance
        int32_t direction = 1;                // 1 or -1 (0 = unspecified)

        MotorType_t motor_type = MOTOR_TYPE_HIGH_CURRENT;

        // Read out max_allowed_current to see max supported value for current_lim.
        #ifdef HOVERBOARD_SETTINGS
        float current_lim = 35.0f;  //[A]
        #else
        float current_lim = 10.0f;  //[A]
        #endif

        // Value used to compute shunt amplifier gains
        float requested_current_range = 60.0f; // [A]

        #ifdef HOVERBOARD_SETTINGS
        float current_control_bandwidth = 100.0f;
        #else
        float current_control_bandwidth = 1000.0f;  // [rad/s]
        #endif

        float inverter_temp_limit_lower = 100;
        float inverter_temp_limit_upper = 120;

        float motor_calibration_velocity = 400.0f;
    };


    enum TimingLog_t : int32_t {
        TIMING_LOG_GENERAL,
        TIMING_LOG_ADC_CB_I,
        TIMING_LOG_ADC_CB_DC,
        TIMING_LOG_MEAS_R,
        TIMING_LOG_MEAS_L,
        TIMING_LOG_ENC_CALIB,
        TIMING_LOG_IDX_SEARCH,
        TIMING_LOG_FOC_VOLTAGE,
        TIMING_LOG_FOC_CURRENT,
        TIMING_LOG_NUM_SLOTS
    };

    enum ArmedState_t : int32_t {
        ARMED_STATE_DISARMED,
        ARMED_STATE_WAITING_FOR_TIMINGS,
        ARMED_STATE_WAITING_FOR_UPDATE,
        ARMED_STATE_ARMED,
    };

    // Make sure these line up with the enumerated types!
    char armed_states_[4][16] = {
        // 0
        "DISARMED",
        "WAIT_TIMING",
        "WAIT_UPDATE",
        "ARMED"
    };


    Motor(const MotorHardwareConfig_t& hw_config,
         const GateDriverHardwareConfig_t& gate_driver_config,
         Config_t& config);

    bool arm();
    void disarm();
    void setup() {
        DRV8301_setup();
    }

    void reset_current_control();
    void update_current_controller_gains();
    void DRV8301_setup();
    bool check_DRV_fault();
    void set_error(Error_t error);
    bool do_checks();
    float get_inverter_temp();
    bool update_thermal_limits();
    float effective_current_lim();
    void log_timing(TimingLog_t log_idx);
    float phase_current_from_adcval(uint32_t ADCValue);
    bool measure_phase_resistance(float test_current, float max_voltage);
    bool measure_phase_inductance(float voltage_low, float voltage_high, float saturating_current, AplhaBetaType_t measurement_axis);
    bool measure_motor_dynamics(bool disarm_after);
    bool run_calibration();
    void reset_timings_stacks();
    bool enqueue_timings(float alphas[6], float betas[6], TimingsType_t timings_type);
    bool FOC_voltage(float v_d, float v_q, float phase, float phase_vel);
    bool FOC_current(float Id_setpoint, float Iq_setpoint, float phase, float phase_vel);
    bool update(float d_current_setpoint, float q_current_setpoint, float phase, float phase_vel);

    const MotorHardwareConfig_t& hw_config_;
    const GateDriverHardwareConfig_t gate_driver_config_;
    Config_t& config_;

    // Motor's parent axis, set by Axis constructor
    Axis* axis_ = nullptr;

    // These are the timings to be applied from the SVM calculator to the PWM timers
    __attribute__((aligned(4))) uint32_t next_timings_stack_[6][3] = {0};

    // These are the timings currently active
    __attribute__((aligned(4))) uint32_t active_timings_stack_[6][3] = {0};

    // If this flag is true then the timings in next_timings_ stack are valid to
    // be loaded into the timer. It must be set true every time new timings are enqeued.
    bool next_timings_valid_ = false;

    // TIM13 Timing log for debugging
    int32_t timing_log_[TIMING_LOG_NUM_SLOTS] = { 0 };

    // Motor error state
    Error_t error_ = ERROR_NONE;

    // Current state of the motor and its PWM
    // Do not write to this variable directly!
    // It is for exclusive use by the safety_critical_... functions.
    ArmedState_t armed_state_ = ARMED_STATE_DISARMED;
    bool is_calibrated_ = config_.pre_calibrated;

    // Sampled current measurements for the motor
    Iph_BC_t current_meas_[3] = { {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f} };
    Iph_BC_t buffered_current_meas_[3] = { {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f} };

    // DC calibration offsets which are applied to the current measurements
    Iph_BC_t DC_calib_ = {0.0f, 0.0f};

    // DRV chip
    DRV8301_Obj gate_driver_;

    // Reverse of the gain of the ADC to Amperes (to be set by DRV8301_setup)
    float phase_current_rev_gain_ = 0.0f;

    ///////////////////////////////////////////////////////////////
    // These are variables used for high-frequency-injection

    // Index of the MIDI modulation table (this should be in midi...)
    uint16_t voltage_modulation_index_ = 0;

    CurrentControl_t current_control_ = {
        .p_gain = 0.0f,        // [V/A] should be auto set after resistance and inductance measurement
        .i_gain = 0.0f,        // [V/As] should be auto set after resistance and inductance measurement
        .p_gain_max = 0.0f,
        .i_gain_max = 0.0f,
        .v_current_control_integral_d = 0.0f,
        .v_current_control_integral_q = 0.0f,
        .Ibus = 0.0f,
        .final_v_alpha = 0.0f,
        .final_v_beta = 0.0f,
        .Iq_setpoint = 0.0f,
        .Iq_measured = 0.0f,
        .Id_setpoint = 0.0f,
        .Id_measured = 0.0f,
        .I_measured_report_filter_k = 1.0f,
        .max_allowed_current = 0.0f,
        .overcurrent_trip_level = 0.0f,
    };

    DRV8301_FaultType_e drv_fault_ = DRV8301_FaultType_NoFault;

    //Local view of DRV registers (initialized by DRV8301_setup)
    DRV_SPI_8301_Vars_t gate_driver_regs_;

    //[A]
    float thermal_current_lim_ = 10.0f;


    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_number_kw(
                &error_,
                property_name = "error"
            ),
            make_protocol_selection_kw(
                (int32_t *) &armed_state_,
                property_name = "armed_state",
                property_option_strings = armed_states_,
                property_option_count = 4,
                property_is_read_only = true
            ),
            make_protocol_number_kw(
                &is_calibrated_,
                property_name = "is_calibrated"
            ),
            make_protocol_number_kw(
                &current_meas_[2].phB,
                property_name = "current_meas_phB",
                property_is_read_only = true
            ),
            make_protocol_number_kw(
                &current_meas_[2].phC,
                property_name = "current_meas_phC",
                property_is_read_only = true
            ),
            make_protocol_number_kw(
                &DC_calib_.phB,
                property_name = "DC_calib_phB"
            ),
            make_protocol_number_kw(
                &DC_calib_.phC,
                property_name = "DC_calib_phC"
            ),
            make_protocol_number_kw(
                &phase_current_rev_gain_,
                property_name = "phase_current_rev_gain"
            ),
            make_protocol_number_kw(
                &thermal_current_lim_,
                property_name = "thermal_current_lim",
                property_is_read_only = true
            ),

            //make_protocol_function("get_inverter_temp", *this, &Motor::get_inverter_temp),

            make_protocol_object("current_control",

                make_protocol_number_kw(
                    &current_control_.p_gain,
                    property_name = "p_gain"
                ),
                make_protocol_number_kw(
                    &current_control_.i_gain,
                    property_name = "i_gain"
                ),
                make_protocol_number_kw(
                    &current_control_.v_current_control_integral_d,
                    property_name = "v_current_control_integral_d"
                ),
                make_protocol_number_kw(
                    &current_control_.v_current_control_integral_q,
                    property_name = "v_current_control_integral_q"
                ),
                make_protocol_number_kw(
                    &current_control_.Ibus,
                    property_name = "Ibus"
                ),
                //make_protocol_number_kw("final_v_alpha", &current_control_.final_v_alpha),
                //make_protocol_number_kw("final_v_beta", &current_control_.final_v_beta),
                make_protocol_number_kw(
                    &current_control_.Iq_setpoint,
                    property_name = "Iq_setpoint"
                ),
                make_protocol_number_kw(
                    &current_control_.Iq_measured,
                    property_name = "Iq_measured"
                ),
                make_protocol_number_kw(
                    &current_control_.Id_setpoint,
                    property_name = "Id_setpoint"
                ),
                make_protocol_number_kw(
                    &current_control_.Id_measured,
                    property_name = "Id_measured"
                ),
                //make_protocol_number_kw("I_measured_report_filter_k", &current_control_.I_measured_report_filter_k),
                make_protocol_number_kw(
                    &current_control_.max_allowed_current,
                    property_name = "max_allowed_current",
                    property_is_read_only = true
                ),
                make_protocol_number_kw(
                    &current_control_.overcurrent_trip_level,
                    property_name = "overcurrent_trip_level",
                    property_is_read_only = true
                )
            ),
            make_protocol_object("gate_driver",
                make_protocol_number_kw(
                    &drv_fault_,
                    property_name = "drv_fault",
                    property_is_read_only = true
                ),
                make_protocol_number_kw(
                    (int32_t *) &gate_driver_regs_.Stat_Reg_1_Value,
                    property_name = "status_reg_1",
                    property_is_read_only = true
                ),
                make_protocol_number_kw(
                    (int32_t *) &gate_driver_regs_.Stat_Reg_2_Value,
                    property_name = "status_reg_2",
                    property_is_read_only = true
                ),
                make_protocol_number_kw(
                    (int32_t *) &gate_driver_regs_.Ctrl_Reg_1_Value,
                    property_name = "ctrl_reg_1",
                    property_is_read_only = true
                ),
                make_protocol_number_kw(
                    (int32_t *) &gate_driver_regs_.Ctrl_Reg_2_Value,
                    property_name = "ctrl_reg_2",
                    property_is_read_only = true
                )
            ),
            /*
            make_protocol_object("timing_log",
                make_protocol_ro_number("TIMING_LOG_GENERAL", &timing_log_[TIMING_LOG_GENERAL]),
                make_protocol_ro_number("TIMING_LOG_ADC_CB_I", &timing_log_[TIMING_LOG_ADC_CB_I]),
                make_protocol_ro_number("TIMING_LOG_ADC_CB_DC", &timing_log_[TIMING_LOG_ADC_CB_DC]),
                make_protocol_ro_number("TIMING_LOG_MEAS_R", &timing_log_[TIMING_LOG_MEAS_R]),
                make_protocol_ro_number("TIMING_LOG_MEAS_L", &timing_log_[TIMING_LOG_MEAS_L]),
                make_protocol_ro_number("TIMING_LOG_ENC_CALIB", &timing_log_[TIMING_LOG_ENC_CALIB]),
                make_protocol_ro_number("TIMING_LOG_IDX_SEARCH", &timing_log_[TIMING_LOG_IDX_SEARCH]),
                make_protocol_ro_number("TIMING_LOG_FOC_VOLTAGE", &timing_log_[TIMING_LOG_FOC_VOLTAGE]),
                make_protocol_ro_number("TIMING_LOG_FOC_CURRENT", &timing_log_[TIMING_LOG_FOC_CURRENT])
            ),*/

            make_protocol_object("config",

                make_protocol_number_kw(
                    &config_.pre_calibrated,
                    property_name = "pre_calibrated"
                ),
                make_protocol_number_kw(
                    &config_.pole_pairs,
                    property_name = "pole_pairs"
                ),
                make_protocol_number_kw(
                    &config_.calibration_current,
                    property_name = "calibration_current"
                ),
                make_protocol_number_kw(
                    &config_.resistance_calib_max_voltage,
                    property_name = "resistance_calib_max_voltage"
                ),
                make_protocol_number_kw(
                    &config_.phase_inductance,
                    property_name = "phase_inductance"
                ),
                make_protocol_number_kw(
                    &config_.phase_resistance,
                    property_name = "phase_resistance"
                ),
                make_protocol_number_kw(
                    &config_.direction,
                    property_name = "direction"
                ),
                make_protocol_selection_kw(
                    (int32_t *) &config_.motor_type,
                    property_name = "motor_type",
                    property_option_strings = motor_types_,
                    property_option_count = 3
                ),
                make_protocol_number_kw(
                    &config_.current_lim,
                    property_name = "current_lim"
                ),
                //make_protocol_number_kw("inverter_temp_limit_lower", &config_.inverter_temp_limit_lower),
                //make_protocol_number_kw("inverter_temp_limit_upper", &config_.inverter_temp_limit_upper),
                make_protocol_number_kw(
                    &config_.requested_current_range,
                    property_name = "requested_current_range"
                ),
                make_protocol_number_kw(
                    &config_.motor_calibration_velocity,
                    property_name = "flux_linkage_calib_velocity"
                )//,
                /*
                make_protocol_number_kw(
                    property_name = "current_control_bandwidth",
                    &config_.current_control_bandwidth,
                    [](void* ctx) { static_cast<Motor*>(ctx)->update_current_controller_gains(); }, this) */
            )
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(Motor::Error_t)

#endif // __MOTOR_HPP
