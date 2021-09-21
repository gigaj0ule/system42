#ifndef __CONTROLLER_HPP
#define __CONTROLLER_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

class Controller {
public:
    enum Error_t: int32_t {
        ERROR_NONE = 0,
        ERROR_OVERSPEED = 0x01,
    };

    // Note: these should be sorted from lowest level of control to
    // highest level of control, to allow "<" style comparisons.
    enum ControlMode_t: int32_t {
        CTRL_MODE_VOLTAGE_CONTROL = 0,
        CTRL_MODE_CURRENT_CONTROL = 1,
        CTRL_MODE_VELOCITY_CONTROL = 2,
        CTRL_MODE_POSITION_CONTROL = 3,
        CTRL_MODE_TRAJECTORY_CONTROL = 4,
        CTRL_MODE_THROTTLE_CONTROL = 5
    };

    // Make sure these line up with the enumerated types!
    char control_modes_[6][16] = {
        "VOLTAGE", 
        "CURRENT",
        "VELOCITY",
        "POSITION",
        "TRAJECTORY",
        "THROTTLE"
    };

    struct Config_t {
        ControlMode_t control_mode = CTRL_MODE_VELOCITY_CONTROL;  //see: Motor_control_mode_t
        float pos_gain = 20.0f;  // [(counts/s) / counts]

        #ifdef HOVERBOARD_SETTINGS
        float vel_gain = 0.025f; // [A/(counts/s)]
        float vel_integrator_gain = 0.01f;
        float vel_derivative_gain = 0.0f;
        #else
        float vel_gain = 5.0f / 10000.0f;  // [A/(counts/s)]
        // float vel_gain = 5.0f / 200.0f; // [A/(rad/s)] <sensorless example>
        float vel_integrator_gain = 10.0f / 10000.0f;  // [A/(counts/s * s)]
        float vel_derivative_gain = 0.0f;        
        #endif
        float vel_limit = 20000.0f;        // [counts/s]
        float vel_limit_tolerance = 1.2f;  // ratio to vel_lim. 0.0f to disable

        #ifdef HOVERBOARD_SETTINGS
        float vel_ramp_rate = 200.0f;  // [(counts/s) / s]
        #else
        float vel_ramp_rate = 10000.0f;  // [(counts/s) / s]
        #endif

        float throttle_brake_current_limit = 50.0f;
        float throttle_acceleration_current_limit = 50.0f;
        float throttle_regen_deceleration_current = 1.0f;
        bool setpoints_in_cpr = false;
        bool vel_limit_error_enabled = false;
    };

    explicit Controller(Config_t& config);
    void reset();
    void set_error(Error_t error);

    void set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward);
    void set_vel_setpoint(float vel_setpoint, float current_feed_forward);
    void set_current_setpoint(float current_setpoint);

    // Trajectory-Planned control
    void move_to_pos(float goal_point);
    void move_incremental(float displacement, bool from_goal_point);
    
    // TODO: make this more similar to other calibration loops
    void start_anticogging_calibration();
    bool anticogging_calibration(float pos_estimate, float vel_estimate);

    bool update(float vel_estimate, float phase, float* current_output, float* voltage_output);

    Config_t& config_;
    Axis* axis_ = nullptr; // set by Axis constructor

    // TODO: anticogging overhaul:
    // - expose selected (all?) variables on protocol
    // - make calibration user experience similar to motor & encoder calibration
    // - use python tools to Fourier transform and write back the smoothed map or Fourier coefficients
    // - make the calibration persistent

    typedef struct {
        int index;
        float *cogging_map;
        bool use_anticogging;
        bool calib_anticogging;
        float calib_pos_threshold;
        float calib_vel_threshold;
    } Anticogging_t;
    
    Anticogging_t anticogging_ = {
        .index = 0,
        .cogging_map = nullptr,
        .use_anticogging = false,
        .calib_anticogging = false,
        .calib_pos_threshold = 1.0f,
        .calib_vel_threshold = 1.0f,
    };

    Error_t error_ = ERROR_NONE;
    // variables exposed on protocol
    float pos_setpoint_ = 0.0f;
    float vel_setpoint_ = 0.0f;
    float throttle_setpoint_  = 0.0f; // Min -1, Max 1
    float brake_setpoint_ = 0.0f; // Min 0, Max 1
    // float vel_setpoint = 800.0f; <sensorless example>
    float vel_integrator_current_ = 0.0f;  // [A]
    float vel_derivative_current_ = 0.0f;  // [A]
    float current_setpoint_ = 0.0f;        // [A]
    float vel_ramp_target_ = 0.0f;
    float dc_bias_current_ = 0.0f; // [A]

    bool vel_ramp_enable_ = true;

    // Used internally by controller
    float vel_error_ = 0.0f;
    float vel_error_last_value_ = 0.0f;
    float vel_setpoint_last_value_  = 0.0f;

    bool direct_mode_ = 0;
    
    //float direct_mode_engage_vel_  = 60.0f;
    //float direct_mode_release_vel_ = 100.0f;
    float direct_mode_hysteresis_state_ = 0.0f;

    //float direct_mode_quiescent_current_ = 3.0f;//-2.0f;
    float direct_mode_direct_current_ = 0.0f;//-2.0f;
    float direct_mode_torque_current_ = 0.0f;
    float direct_mode_phase_ = 0;
    float direct_mode_vel_ = 0;
    float direct_mode_position_ = 0;

    uint32_t traj_start_loop_count_ = 0;

    float goal_point_ = 0.0f;

    float effective_current_limit_ = 0.0f;

    // Communication protocol definitions
    auto make_protocol_definitions() {

        return make_protocol_member_list(
            /*
            make_protocol_number("error", &error_),
            make_protocol_number("pos_setpoint", &pos_setpoint_),
            make_protocol_number("vel_setpoint", &vel_setpoint_),
            make_protocol_ro_number("vel_error", &vel_error_),
            make_protocol_number("vel_integrator_current", &vel_integrator_current_),
            make_protocol_number("current_setpoint", &current_setpoint_),
            make_protocol_number("throttle_setpoint", &throttle_setpoint_),
            make_protocol_number("brake_setpoint", &brake_setpoint_),
            make_protocol_number("vel_ramp_target", &vel_ramp_target_),
            make_protocol_number("vel_ramp_enable", &vel_ramp_enable_),
            make_protocol_number("dc_bias_current", &dc_bias_current_),
            make_protocol_object("config",
                make_protocol_selection("control_mode", (int32_t *) &config_.control_mode, nullptr, control_modes_, 6),
                make_protocol_number("pos_gain", &config_.pos_gain),
                make_protocol_number("vel_gain", &config_.vel_gain),
                make_protocol_number("vel_integrator_gain", &config_.vel_integrator_gain),
                //make_protocol_number("vel_derivative_gain", &config_.vel_derivative_gain),
                make_protocol_number("vel_limit", &config_.vel_limit),
                make_protocol_number("vel_limit_tolerance", &config_.vel_limit_tolerance),
                make_protocol_number("vel_limit_error_enabled", &config_.vel_limit_error_enabled),
                make_protocol_number("vel_ramp_rate", &config_.vel_ramp_rate),
                make_protocol_number("throttle_brake_current_limit", &config_.throttle_brake_current_limit),
                make_protocol_number("throttle_acceleration_current_limit", &config_.throttle_acceleration_current_limit),
                make_protocol_number("throttle_regen_deceleration_current", &config_.throttle_regen_deceleration_current),
                make_protocol_number("setpoints_in_cpr", &config_.setpoints_in_cpr)
            ),
            make_protocol_function("set_vel_setpoint", *this, &Controller::set_vel_setpoint,
                "vel_setpoint", "current_feed_forward"),
            make_protocol_function("set_current_setpoint", *this, &Controller::set_current_setpoint,
                                   "current_setpoint")
            */
        );
    }
};

DEFINE_ENUM_FLAG_OPERATORS(Controller::Error_t)

#endif // __CONTROLLER_HPP
