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
        AXIS_STATE_STARTUP_SEQUENCE = 2,    //<! the actual sequence is defined by the config.startup_... flags
        AXIS_STATE_ACTIVE = 3,              //<! Normal Operation
        AXIS_STATE_CALIBRATION = 4          //<! run calibration
    };

    // Make sure these line up with the enumerated types!
    char axis_states_[5][16] = {
        // 0
        "UNDEFINED", 
        "IDLE",
        "STARTUP_SEQ",
        "ACTIVE",
        "CALIBRATE",
    };

    struct Config_t {
        bool startup_motor_calibration = false;   //<! run motor calibration at startup, skip otherwise
        bool startup_inverter_automatically = true; //<! enable sensorless control after calibration/startup
        bool enable_step_dir = false; //<! enable step/dir input after calibration
                                    //   For M0 this has no effect if enable_uart is true
        float counts_per_step = 2.0f;

        float watchdog_timeout = 0.0f; // [s] (0 disables watchdog)

        // Defaults loaded from hw_config in load_configuration in main.cpp
        int32_t step_gpio_pin = 0;
        int32_t dir_gpio_pin = 0;
    };

    enum thread_signals {
        M_SIGNAL_PH_CURRENT_MEAS = 1u << 0
    };

    Axis(const AxisHardwareConfig_t& hw_config,
            Config_t& config,
            SensorlessEstimator& sensorless_estimator,
            Controller& controller,
            Motor& motor);

    void setup();
    void start_thread();
    void signal_current_meas();
    bool wait_for_current_meas();

    void update_watchdog_settings();

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

    bool run_inverter_loop();

    bool run_idle_loop();

    void run_state_machine_loop();

    const AxisHardwareConfig_t& hw_config_;
    Config_t& config_;

    SensorlessEstimator& sensorless_estimator_;
    Controller& controller_;
    Motor& motor_;

    osThreadId thread_id_;
    volatile bool thread_id_valid_ = false;


    // Phase angle of inverter
    float phase_angle_ = 0.0f;
    float base_frequency_ = 60.0f;

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

    // watchdog
    int32_t watchdog_reset_value_ = 0; //computed from config_.watchdog_timeout in update_watchdog_settings()
    int32_t watchdog_current_value_= 0;

    // Communication protocol definitions
    auto make_protocol_definitions() {
        return make_protocol_member_list(
            
            // Error status
            make_protocol_number_kw(&error_, property_name = "error"),

            // Operational state
            make_protocol_selection_kw((int32_t *) &current_state_, property_name = "current_state", property_option_strings = axis_states_, property_option_count = 5, property_is_read_only = true),
            make_protocol_selection_kw((int32_t *) &requested_state_, property_name = "requested_state", property_option_strings = axis_states_, property_option_count = 5),

            // Debugging
            make_protocol_number_kw(&loop_counter_, property_name = "loop_counter", property_is_read_only = true),

            make_protocol_object("config",

                // Startup settings
                //make_protocol_number_kw(&config_.startup_motor_calibration, property_name = "startup_motor_calibration"),
                make_protocol_number_kw(&config_.startup_inverter_automatically, property_name = "startup_inverter_automatically")

            ),

            // Explode modulator settings
            make_protocol_object("modulator", motor_.make_protocol_definitions())
            
            //,
            //make_protocol_object("controller", controller_.make_protocol_definitions())
            //make_protocol_object("trap_traj", trap_.make_protocol_definitions()),
            //make_protocol_function("watchdog_feed", *this, &Axis::watchdog_feed)
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(Axis::Error_t)

#endif /* __AXIS_HPP */
