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

    enum AplhaBetaType_t  {
        TYPE_ALPHA = 0,
        TYPE_BETA = 1
    };

    struct Iph_BC_t {
        float phB;
        float phC;
    };

    struct CurrentControl_t {
        float I_integral; // [V]
        float Ibus; // DC bus current [A]
        // Voltage applied at end of cycle:
        float I_b_measured; // [A]
        float I_c_measured; // [A]
        float I_magnitude;
        float I_total;
        float I_error;
        float I_measured_report_filter_k;
        float max_allowed_current; // [A]
        float overcurrent_trip_level; // [A]
    };

    struct VoltageControl_t {
        float V_out_measured; // [V]
        float V_out_measured_report_filter_k;
        float V_out_magnitude; // [V]
        float V_out_error;
        float V_integral;
    };

    float saved_steady_state_v_current_control_integral_d_;
    float saved_steady_state_v_current_control_integral_q_;  

    // NOTE: for gimbal motors, all units of A are instead V.
    // example: vel_gain is [V/(count/s)] instead of [A/(count/s)]
    // example: current_lim and calibration_current will instead determine the maximum voltage applied to the motor.
    struct Config_t {
        bool pre_calibrated = true; // can be set to true to indicate that all values here are valid
        
        /*
        #ifdef HOVERBOARD_SETTINGS
        int32_t pole_pairs = 15;
        #else
        int32_t pole_pairs = 7;
        #endif
        */

        float calibration_current = 5.0f;    // [A]
        float resistance_calib_max_voltage = 5.0f; // [V] - You may need to increase this if this voltage isn't sufficient to drive calibration_current through the motor.
        
        float phase_inductance = 0.01f;        // to be set by measure_phase_inductance
        float phase_resistance = 0.01f;        // to be set by measure_phase_resistance
                
        // Read out max_allowed_current to see max supported value for current_lim.
        float current_lim = 35.0f;  //[A]
        
        float cbc_current_lim = 5.0f;
        
        // Value used to compute shunt amplifier gains        
        float current_control_bandwidth = 60.0f;  // [rad/s]

        float inverter_temp_limit_lower = 100;
        float inverter_temp_limit_upper = 120;

        float output_voltage_rms = 15.0f;

        float current_limit_cycle_count = 5.0f;

        float I_p_gain = 1.0f;
        float I_i_gain = 50.0f;

        float I_setpoint = 1.0f;
        float V_setpoint = 15.0f;

        float V_p_gain = 1.0f;
        float V_i_gain = 50.0f;

        float voltage_control_bandwidth = 60.0f;
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
        //DRV8301_setup();
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
    float output_voltage_from_adcval(uint32_t ADCValue);
    bool measure_phase_resistance(float test_current, float max_voltage);
    bool measure_phase_inductance(float voltage_low, float voltage_high, float saturating_current, AplhaBetaType_t measurement_axis);
    bool measure_motor_dynamics(bool disarm_after);    
    bool run_calibration();
    void reset_timings_stacks();
    
    bool enqueue_timings(float b_phase[6], float c_phase[6]);
    bool update_controller(float vel_estimate, float phase, float* current_output, float* voltage_modulation_magnitude);
    bool update_modulations(float modulation_magnitude, float phase, float phase_vel);

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

    // Sampled current measurements for the motor ((I_b, I_c), (I_b, I_c), (I_b, I_c))
    Iph_BC_t current_meas_[6] = { {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f} };
    Iph_BC_t buffered_current_meas_[6] = { {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f}, {0.0f, 0.0f} };

    float buffered_output_voltage_meas_[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    float output_voltages_[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // DC calibration offsets which are applied to the current measurements
    Iph_BC_t DC_calib_ = {0.0f, 0.0f};

    // DC calibration applied to output voltage measurement
    float V_out_DC_calib_ = 0.0f;

    // DRV chip
    DRV8301_Obj gate_driver_; 

    // Reverse of the gain of the ADC to Amperes (to be set by DRV8301_setup)
    float phase_current_rev_gain_ = 1.0f / 50.0f; 

    float output_voltage_rev_gain_ = 333.33333f;

    ///////////////////////////////////////////////////////////////
    // These are variables used for high-frequency-injection

    // Index of the MIDI modulation table (this should be in midi...)
    uint16_t voltage_modulation_index_ = 0;

    CurrentControl_t current_control_ = {
        .I_integral = 0.0f,
        .Ibus = 0.0f,
        .I_b_measured = 0.0f,
        .I_c_measured = 0.0f,
        .I_magnitude = 0.0f,
        .I_total = 0.0f,
        .I_error = 0.0f,
        .I_measured_report_filter_k = 0.1f,
        .max_allowed_current = 45.0f,
        .overcurrent_trip_level = 50.0f
    };

    VoltageControl_t voltage_control_ = {
        .V_out_measured = 0.0f,
        .V_out_measured_report_filter_k = 0.1f,
        .V_out_magnitude = 0.0f,
        .V_out_error = 0.0f,
        .V_integral = 0.0f
    };

    DRV8301_FaultType_e drv_fault_ = DRV8301_FaultType_NoFault;
    
    //Local view of DRV registers (initialized by DRV8301_setup)
    DRV_SPI_8301_Vars_t gate_driver_regs_; 
    
    //[A]
    float thermal_current_lim_ = 10.0f;  

    float V_out_ref_adc_value_ = 0.0f;


    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(

            // Error status
            make_protocol_number_kw(&error_, property_name = "error"),
            
            // Present modulator state
            make_protocol_selection_kw((int32_t *) &armed_state_, property_name = "armed_state", property_option_strings = armed_states_, property_option_count = 4),
            
            // Phase measurements (debugging)
            make_protocol_number_kw(&current_control_.I_b_measured, property_name = "I_b_measured", property_units = "A", property_is_read_only = true),
            make_protocol_number_kw(&current_control_.I_c_measured, property_name = "I_c_measured",  property_units = "A", property_is_read_only = true),
            
            make_protocol_number_kw(
                &voltage_control_.V_out_measured, 
                property_name = "V_out_measured", 
                property_units = "V", 
                property_is_read_only = true
            ),

            make_protocol_number_kw(&current_control_.I_measured_report_filter_k, property_name = "I_measured_report_filter_k"),


            // Phase offsets (debugging)
            make_protocol_number_kw(&DC_calib_.phB, property_name = "DC_cal_I_b", property_units = "A", property_is_read_only = true),
            make_protocol_number_kw(&DC_calib_.phC, property_name = "DC_cal_I_c", property_units = "A", property_is_read_only = true),
            
            make_protocol_object("controller",

                // I control loop information
                make_protocol_number_kw(&current_control_.I_magnitude, property_name = "I_magnitude", property_units = "A", property_is_read_only = true),
                make_protocol_number_kw(&current_control_.I_error, property_name = "I_error", property_units = "A", property_is_read_only = true),
                make_protocol_number_kw(&current_control_.I_integral, property_name = "I_integral", property_is_read_only = true),
                
                // I reporting
                make_protocol_number_kw(&current_control_.Ibus, property_name = "I_bus", property_units = "A", property_is_read_only = true),
                                
                // Info about limits (todo: why?)
                make_protocol_number_kw(&current_control_.max_allowed_current, property_name = "max_allowed_current", property_units = "A", property_is_read_only = true),
                make_protocol_number_kw(&current_control_.overcurrent_trip_level, property_name = "overcurrent_trip_level", property_units = "A", property_is_read_only = true),

                // V control loop information
                make_protocol_number_kw(
                    &voltage_control_.V_out_magnitude, 
                    property_name = "V_out_magnitude", 
                    property_units = "V", 
                    property_is_read_only = true
                ),

                make_protocol_number_kw(
                    &voltage_control_.V_out_error, 
                    property_name = "V_out_error", 
                    property_units = "V", 
                    property_is_read_only = true
                ),

                make_protocol_number_kw(
                    &voltage_control_.V_integral, 
                    property_name = "V_integral", 
                    property_is_read_only = true
                ),

                make_protocol_object("config",

                    // V Setpoint
                    make_protocol_number_kw(
                        &config_.V_setpoint, 
                        property_name = "V_setpoint", 
                        property_units = "V",
                        property_is_non_volatile = true
                    ),

                    // Todo: V control loop

                    // I Setpoint
                    make_protocol_number_kw(
                        &config_.I_setpoint, 
                        property_name = "I_setpoint", 
                        property_units = "A",
                        property_is_non_volatile = true
                    ),

                    // I control loop gains
                    make_protocol_number_kw(
                        &config_.I_p_gain, 
                        property_name = "current_proportional_gain", 
                        property_units = "V/A",
                        property_is_non_volatile = true
                    ),

                    make_protocol_number_kw(
                        &config_.I_i_gain, 
                        property_name = "current_integral_gain", 
                        property_units = "V/(A s)",
                        property_is_non_volatile = true
                    ),

                    make_protocol_number_kw(
                        &config_.current_control_bandwidth, 
                        property_name = "current_control_bandwidth", 
                        property_units = "Hz",
                        property_is_non_volatile = true
                    ),

                    // V control loop
                    make_protocol_number_kw(
                        &config_.voltage_control_bandwidth,
                        property_name = "voltage_control_bandwidth",
                        property_units = "Hz",
                        property_is_non_volatile = true
                    ),

                    make_protocol_number_kw(
                        &config_.V_p_gain,
                        property_name = "voltage_proportional_gain",
                        property_units = "V/V",
                        property_is_non_volatile = true
                    ),

                    make_protocol_number_kw(
                        &config_.V_i_gain,
                        property_name = "voltage_integral_gain",
                        property_units = "V(V s)",
                        property_is_non_volatile = true
                    ),

                    // Limits
                    make_protocol_number_kw(
                        &config_.cbc_current_lim, 
                        property_name = "cycle_by_cycle_current_limit", 
                        property_units = "A",
                        property_is_non_volatile = true
                    ),

                    make_protocol_number_kw(
                        &config_.current_lim, 
                        property_name = "absolute_current_limit", 
                        property_units = "A",
                        property_is_non_volatile = true
                    )
                )
            )
            
            /*,
            // Gate driver
            make_protocol_object("gate_driver",
                make_protocol_number_kw(&drv_fault_, property_name = "drv_fault", property_is_read_only = true),
                make_protocol_number_kw(&phase_current_rev_gain_, property_name = "phase_current_rev_gain", property_is_read_only = true),
                make_protocol_number_kw((int32_t *) &gate_driver_regs_.Stat_Reg_1_Value, property_name = "status_reg_1", property_is_read_only = true),
                make_protocol_number_kw((int32_t *) &gate_driver_regs_.Stat_Reg_2_Value, property_name = "status_reg_2", property_is_read_only = true),
                make_protocol_number_kw((int32_t *) &gate_driver_regs_.Ctrl_Reg_1_Value, property_name = "ctrl_reg_1", property_is_read_only = true),
                make_protocol_number_kw((int32_t *) &gate_driver_regs_.Ctrl_Reg_2_Value, property_name = "ctrl_reg_2", property_is_read_only = true)
            )*/
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(Motor::Error_t)

#endif // __MOTOR_HPP
