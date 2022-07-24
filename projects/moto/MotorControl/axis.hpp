#ifndef __AXIS_HPP
#define __AXIS_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

class Axis {
public:
    enum Error_t : int32_t {
        ERROR_NONE = 0x00,
        ERROR_INVALID_STATE = 0x01, //<! an invalid state was requested
        ERROR_DC_BUS_UNDER_VOLTAGE = 0x02,
        ERROR_DC_BUS_OVER_VOLTAGE = 0x04,
        ERROR_CURRENT_MEASUREMENT_TIMEOUT = 0x08,
        ERROR_BRAKE_RESISTOR_DISARMED = 0x10, //<! the brake resistor was unexpectedly disarmed
        ERROR_MOTOR_DISARMED = 0x20, //<! the motor was unexpectedly disarmed
        ERROR_MOTOR_FAILED = 0x40, // Go to motor.hpp for information, check odrvX.axisX.motor.error for error value 
        ERROR_SENSORLESS_ESTIMATOR_FAILED = 0x80,
        ERROR_ENCODER_FAILED = 0x100, // Go to encoder.hpp for information, check odrvX.axisX.encoder.error for error value
        ERROR_CONTROLLER_FAILED = 0x200,
        ERROR_POS_CTRL_DURING_SENSORLESS = 0x400,
        ERROR_WATCHDOG_TIMER_EXPIRED = 0x800,
    };

    // Warning: Do not reorder these enum values.
    // The state machine uses ">" comparision on them.
    enum State_t : int32_t {
        AXIS_STATE_UNDEFINED = 0,           //<! will fall through to idle
        AXIS_STATE_IDLE = 1,                //<! disable PWM and do nothing
        AXIS_STATE_STARTUP_SEQUENCE = 2, //<! the actual sequence is defined by the config.startup_... flags
        AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,   //<! run all calibration procedures, then idle
        AXIS_STATE_MOTOR_CALIBRATION = 4,   //<! run motor calibration
        AXIS_STATE_SENSORLESS_CONTROL = 5,  //<! run sensorless control
        AXIS_STATE_ENCODER_INDEX_SEARCH = 6, //<! run encoder index search
        AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
        AXIS_STATE_CLOSED_LOOP_CONTROL = 8,  //<! run closed loop control
        AXIS_STATE_LOCKIN_SPIN = 9,       //<! run lockin spin
        AXIS_STATE_ENCODER_DIR_FIND = 10,
        AXIS_STATE_HFI_CALIBRATION = 11,
    };

    // Make sure these line up with the enumerated types!
    char axis_states_[12][16] = {
        // 0
        "UNDEFINED", 
        "IDLE",
        "STARTUP_SEQ",
        "FULL_CALIBRATE",
        "MOTOR_CALIB",
        "SENSORLESS",
        "ENCODR_INDEX",
        "ENCODR_OFFSET",
        "CLOSED_LOOP_CTL",
        "LOCKIN_SPIN",
        "ENCODR_DIR_FIND",
        "HFI_CALIBRATION"
    };

    struct LockinConfig_t {
        float current = 5.0f;           // [A]
        float ramp_time = 0.4f;          // [s]
        float ramp_distance = 1 * M_PI;  // [rad]
        float accel = 20.0f;     // [rad/s^2]
        float vel = 40.0f; // [rad/s]
        float finish_distance = 100.0f;  // [rad]

        float initial_phase = 0.0f;

        bool finish_on_vel = false;                
        bool finish_on_distance = false;
        bool finish_on_enc_idx = false;
        bool finish_on_flag = false;
    };

    struct Config_t {
        bool startup_motor_calibration = true;   //<! run motor calibration at startup, skip otherwise
        //bool startup_motor_calibration = false;   //<! run motor calibration at startup, skip otherwise
        bool startup_encoder_index_search = false; //<! run encoder index search after startup, skip otherwise
                                                // this only has an effect if encoder.config.use_index is also true
        bool startup_encoder_offset_calibration = false; //<! run encoder offset calibration after startup, skip otherwise
        bool startup_closed_loop_control = false; //<! enable closed loop control after calibration/startup
        bool startup_sensorless_control = true; //<! enable sensorless control after calibration/startup
        bool enable_step_dir = false; //<! enable step/dir input after calibration
                                    //   For M0 this has no effect if enable_uart is true
        float counts_per_step = 2.0f;

        float watchdog_timeout = 0.0f; // [s] (0 disables watchdog)

        // Defaults loaded from hw_config in load_configuration in main.cpp
        int32_t step_gpio_pin = 0;
        int32_t dir_gpio_pin = 0;

        // Sensorless spin-up config
        LockinConfig_t lockin;
    };

    enum thread_signals {
        M_SIGNAL_PH_CURRENT_MEAS = 1u << 0
    };

    enum LockinState_t : int32_t {
        LOCKIN_STATE_INACTIVE,
        LOCKIN_STATE_RAMP,
        LOCKIN_STATE_ACCELERATE,
        LOCKIN_STATE_CONST_VEL,
    };

    // Make sure these line up with the enumerated types!
    char lockin_states_[4][16] = {
        // 0
        "INACTIVE", 
        "RAMP",
        "ACCELERATE",
        "CONST_VELOCITY"
    };

    Axis(const AxisHardwareConfig_t& hw_config,
            Config_t& config,
            Encoder& encoder,
            SensorlessEstimator& sensorless_estimator,
            Controller& controller,
            Motor& motor,
            TrapezoidalTrajectory& trap);

    void setup();
    void start_thread();
    void signal_current_meas();
    bool wait_for_current_meas();

    void step_cb();
    void set_step_dir_active(bool enable);
    void decode_step_dir_pins();
    void update_watchdog_settings();

    static void load_default_step_dir_pin_config(
        const AxisHardwareConfig_t& hw_config, Config_t* config);

    bool check_DRV_fault();
    bool check_PSU_brownout();
    bool do_checks();
    bool do_updates();

    void watchdog_feed();
    bool watchdog_check();


    // True if there are no errors
    bool inline check_for_errors() {
        return error_ == ERROR_NONE;
    }

    // @brief Runs the specified update handler at the frequency of the current measurements.
    //
    // The loop runs until one of the following conditions:
    //  - update_handler returns false
    //  - the current measurement times out
    //  - the health checks fail (brownout, driver fault line)
    //  - update_handler doesn't update the modulation timings in time
    //    This criterion is ignored if current_state is AXIS_STATE_IDLE
    //
    // If update_handler is going to update the motor timings, you must call motor.arm()
    // shortly before this function.
    //
    // If the function returns, it is guaranteed that error is non-zero, except if the cause
    // for the exit was a negative return value of update_handler or an external
    // state change request (requested_state != AXIS_STATE_DONT_CARE).
    // Under all exit conditions the motor is disarmed and the brake current set to zero.
    // Furthermore, if the update_handler does not set the phase voltages in time, they will
    // go to zero.
    //
    // @tparam T Must be a callable type that takes no arguments and returns a bool
    template<typename T>
    void run_control_loop(const T& update_handler) {
        while (requested_state_ == AXIS_STATE_UNDEFINED) {
            // look for errors at axis level and also all subcomponents
            bool checks_ok = do_checks();
            // Update all estimators
            // Note: updates run even if checks fail
            bool updates_ok = do_updates(); 

            // make sure the watchdog is being fed. 
            bool watchdog_ok = watchdog_check();
            
            if (!checks_ok || !updates_ok || !watchdog_ok) {
                // It's not useful to quit idle since that is the safe action
                // Also leaving idle would rearm the motors
                if (current_state_ != AXIS_STATE_IDLE)
                    break;
            }

            // Run main loop function, defer quitting for after wait
            // TODO: change arming logic to arm after waiting
            bool main_continue = update_handler();

            // Check we meet deadlines after queueing
            ++loop_counter_;
            if(loop_counter_ % current_meas_hz == 0) {
                loop_seconds_counter_ ++;
            }

            // Wait until the current measurement interrupt fires
            if (!wait_for_current_meas()) {
                // maybe the interrupt handler is dead, let's be
                // safe and float the phases
                safety_critical_disarm_motor_pwm(motor_);
                update_brake_current();
                error_ |= ERROR_CURRENT_MEASUREMENT_TIMEOUT;
                break;
            }

            if (!main_continue)
                break;
        }
    }

    bool run_lockin_spin(bool skip_accelerate);
    bool run_sensorless_control_loop();
    bool run_closed_loop_control_loop();
    bool run_idle_loop();

    void run_state_machine_loop();

    const AxisHardwareConfig_t& hw_config_;
    Config_t& config_;

    Encoder& encoder_;
    SensorlessEstimator& sensorless_estimator_;
    Controller& controller_;
    Motor& motor_;
    TrapezoidalTrajectory& trap_;

    osThreadId thread_id_;
    volatile bool thread_id_valid_ = false;

    // variables exposed on protocol
    Error_t error_ = ERROR_NONE;
    bool step_dir_active_ = false; // auto enabled after calibration, based on config.enable_step_dir

    // updated from config in constructor, and on protocol hook
    GPIO_TypeDef* step_port_;
    int32_t step_pin_;
    GPIO_TypeDef* dir_port_;
    int32_t dir_pin_;

    State_t requested_state_ = AXIS_STATE_STARTUP_SEQUENCE;
    State_t task_chain_[10] = { AXIS_STATE_UNDEFINED };
    State_t& current_state_ = task_chain_[0];
    int64_t loop_counter_ = 0;
    int64_t loop_seconds_counter_ = 0;
    LockinState_t lockin_state_ = LOCKIN_STATE_INACTIVE;
    bool lockin_finish_now_flag_ = false;

    // watchdog
    int32_t watchdog_reset_value_ = 0; //computed from config_.watchdog_timeout in update_watchdog_settings()
    int32_t watchdog_current_value_= 0;

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            
            make_protocol_number_kw(
                &error_,
                property_name = "error"
            ),
            make_protocol_number_kw(
                &step_dir_active_,
                property_name = "step_dir_active",
                property_is_read_only = true
            ),            
            make_protocol_selection_kw(
                (int32_t *) &current_state_, 
                property_name = "current_state", 
                property_option_strings = axis_states_,
                property_option_count = 12,
                property_is_read_only = true
            ),
            make_protocol_selection_kw(
                (int32_t *) &requested_state_, 
                property_name = "requested_state", 
                property_option_strings = axis_states_,
                property_option_count = 12
            ),
            make_protocol_number_kw(
                &loop_counter_,
                property_name = "loop_counter", 
                property_is_read_only = true
            ),
            make_protocol_selection_kw(
                (int32_t *) &lockin_state_, 
                property_name = "lockin_state", 
                property_option_strings = lockin_states_, 
                property_option_count = 4,
                property_is_read_only = true
            ),

            make_protocol_object("config",

                make_protocol_number_kw(
                    &config_.startup_motor_calibration,
                    property_name = "startup_motor_calibration" 
                ),
                //make_protocol_number_kw("startup_encoder_index_search", &config_.startup_encoder_index_search),
                make_protocol_number_kw(
                    &config_.startup_encoder_offset_calibration,
                    property_name = "startup_encoder_offset_calibration"
                ),
                make_protocol_number_kw(
                    &config_.startup_closed_loop_control,
                    property_name = "startup_closed_loop_control"
                ),
                make_protocol_number_kw(
                    &config_.startup_sensorless_control,
                    property_name = "startup_sensorless_control"
                ),
                //make_protocol_number_kw("enable_step_dir", &config_.enable_step_dir),
                //make_protocol_number_kw("counts_per_step", &config_.counts_per_step),
                /*make_protocol_number_kw("watchdog_timeout", &config_.watchdog_timeout, nullptr, 0.0f, 0.0f,
                    [](void* ctx) { static_cast<Axis*>(ctx)->update_watchdog_settings(); }, this),
                make_protocol_number_kw("step_gpio_pin", &config_.step_gpio_pin, nullptr, (int32_t) 0, (int32_t) 0,
                    [](void* ctx) { static_cast<Axis*>(ctx)->decode_step_dir_pins(); }, this),
                make_protocol_number_kw("dir_gpio_pin", &config_.dir_gpio_pin, nullptr, (int32_t) 0, (int32_t) 0,
                    [](void* ctx) { static_cast<Axis*>(ctx)->decode_step_dir_pins(); }, this),
                */
                make_protocol_object("lockin",
                    make_protocol_number_kw(
                        &config_.lockin.current,
                        property_name = "current"
                    ),
                    make_protocol_number_kw(
                        &config_.lockin.ramp_time,
                        property_name = "ramp_time"
                    ),
                    make_protocol_number_kw(
                        &config_.lockin.ramp_distance,
                        property_name = "ramp_distance"
                    ),
                    make_protocol_number_kw(
                        &config_.lockin.accel,
                        property_name = "accel"
                    ),
                    make_protocol_number_kw(
                        &config_.lockin.vel,
                        property_name = "vel"
                    ),
                    make_protocol_number_kw(
                        &config_.lockin.finish_distance,
                        property_name = "finish_distance"
                    ),
                    make_protocol_number_kw(
                        &config_.lockin.initial_phase,
                        property_name = "initial_phase"
                    ),                    
                    make_protocol_number_kw(
                        &config_.lockin.finish_on_vel,
                        property_name = "finish_on_vel"
                    ),
                    make_protocol_number_kw(
                        &config_.lockin.finish_on_distance,
                        property_name = "finish_on_distance"
                    ),
                    make_protocol_number_kw(
                        &config_.lockin.finish_on_enc_idx,
                        property_name = "finish_on_enc_idx"
                    )
                )
            ),

            make_protocol_object("motor", motor_.make_protocol_definitions()),
            make_protocol_object("controller", controller_.make_protocol_definitions()),
            make_protocol_object("encoder", encoder_.make_protocol_definitions()),
            make_protocol_object("sensorless_estimator", sensorless_estimator_.make_protocol_definitions()),
            make_protocol_object("trap_traj", trap_.make_protocol_definitions())
            //make_protocol_function("watchdog_feed", *this, &Axis::watchdog_feed)
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(Axis::Error_t)

#endif /* __AXIS_HPP */
