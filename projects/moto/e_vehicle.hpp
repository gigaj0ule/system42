#ifndef __E_VEHICLE_HPP
#define __E_VEHICLE_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

class E_Vehicle {
public:
    enum Error_t : int32_t {
        ERROR_NONE = 0,
        ERROR_FEATURE_UNIMPLEMENTED = 0x01,
        ERROR_MODE_NOT_ENABLED      = 0x02 
    };

    Error_t error_ = ERROR_NONE;

    enum DriveMode_t : int32_t {
        DRIVE_MODE_UNDEFINED = 0,
        DRIVE_MODE_PARK = 1,
        DRIVE_MODE_NEUTRAL = 2,
        DRIVE_MODE_FREEWHEEL = 3,
        DRIVE_MODE_DRIVE = 4,
        DRIVE_MODE_REVERSE = 5,       
    };

    enum DriveConfig_t : int32_t {
        DRIVE_CONFIG_1WD = 0,
        DRIVE_CONFIG_2WD_INLINE = 1,
        DRIVE_CONFIG_2WD_AXEL = 2,
        DRIVE_CONFIG_4WD_AXEL = 3
    };

    enum MasterAxis_t : int32_t {
        MASTER_AXIS_0 = 0,
        MASTER_AXIS_1 = 1
    };

    struct ControllerParams_t{
        float drive_vel_gain                = 0.05f;
        float drive_vel_integrator_gain     = 0.1f;
        float drive_vel_derivative_gain     = 0.0f;

        float park_vel_gain                 = 0.11f;
        float park_vel_integrator_gain      = 1.5f;
        float park_vel_derivative_gain      = 0.0f;
        float park_current_limit            = 40.0f;

        float throttle_current_limit        = 50.0f;
        float brake_current_limit           = 40.0f;
        float regen_deceleration_current    = 1.0f;
        float max_current_limit             = 60.0f;
    };

    struct PhysicsParams_t
    {
        DriveConfig_t drive_config  = DRIVE_CONFIG_1WD;
        MasterAxis_t master_axis    = MASTER_AXIS_0;
        float axle_width            = 1.0f;
    };

    struct Config_t {
        ControllerParams_t controller_params;
        PhysicsParams_t physics;

        bool ev_controller_enabled = 0;
        bool abs_enabled = 0;

        float gas_pedal_adc_min_voltage = 0.05f;
        float gas_pedal_adc_max_voltage = 1.0f;

        float brake_pedal_adc_min_voltage = 0.05f;
        float brake_pedal_adc_max_voltage = 1.0f;

        osPriority thread_priority = osPriorityAboveNormal;
    };

    Config_t& config_;

    struct ParkingBrake_t
    {
        bool engaged = false;
        bool last_state = false;
        DriveMode_t cached_drive_mode  = DRIVE_MODE_UNDEFINED;
    };
    
    struct CruiseControl_t 
    {
        bool engaged = false;
        float setpoint = 0.0f;
        float gas_pedal_cached_value = 0.0f;
    };

    struct States_t
    {
        DriveMode_t drive_mode = DRIVE_MODE_DRIVE;
        DriveMode_t last_drive_mode = DRIVE_MODE_UNDEFINED;

        float gas_pedal_value = 0;

        float brake_pedal_value = 0;
        bool  brake_locked = 0;
        bool  brake_locked_last_state = 0;

        ParkingBrake_t parking_brake;
        CruiseControl_t cruise_control;
        
        bool configured_gpio_pins_for_ev = false;

        uint8_t number_of_active_axes   = 0;
        uint8_t master_axis             = 0;
        uint8_t subordinate_axis        = 0;
    };
        
    States_t states_;

    E_Vehicle(Config_t& config);

    bool update(void);

    void toggle_state_cruise_control(bool start_debounce_timer);
    void toggle_state_parking_brake(bool start_debounce_timer);

    auto make_protocol_definitions() {
        return make_protocol_member_list(
            make_protocol_number_kw(
                &error_,
                property_name = "error"
            ),
            make_protocol_number_kw(
                &states_.drive_mode,
                property_name = "drive_mode"
            ),
            make_protocol_number_kw(
                &states_.gas_pedal_value,
                property_name = "gas_pedal_value"
            ),
            make_protocol_number_kw(
                &states_.brake_pedal_value,
                property_name = "brake_pedal_value"
            ),
            make_protocol_number_kw(
                &states_.parking_brake.engaged,
                property_name = "parking_brake_engaged"
            ),
            make_protocol_number_kw(
                &states_.cruise_control.engaged,
                property_name = "cruise_control_engaged"
            ),
            make_protocol_number_kw(
                &states_.cruise_control.setpoint,
                property_name = "cruise_control_setpoint"
            ),
            
            make_protocol_object("config",

                make_protocol_number_kw(
                    &config_.ev_controller_enabled,
                    property_name = "ev_controller_enabled"
                ),
                make_protocol_number_kw( 
                    &config_.abs_enabled,
                    property_name = "abs_enabled"
                ),
                make_protocol_number_kw(
                    &config_.gas_pedal_adc_min_voltage,
                    property_name = "gas_adc_min_voltage"
                ),
                make_protocol_number_kw(
                    &config_.gas_pedal_adc_max_voltage,
                    property_name = "gas_adc_max_voltage"
                ),
                make_protocol_number_kw(
                    &config_.brake_pedal_adc_min_voltage,
                    property_name = "brake_adc_min_voltage"
                ),
                make_protocol_number_kw(
                    &config_.brake_pedal_adc_max_voltage,
                    property_name = "brake_adc_max_voltage"
                ),

                make_protocol_object("physics", 

                    make_protocol_number_kw(
                        &config_.physics.drive_config,
                        property_name = "drive_config"
                    ),
                    make_protocol_number_kw(
                        &config_.physics.master_axis,
                        property_name = "master_axis"
                    ),
                    make_protocol_number_kw(
                        &config_.physics.axle_width,
                        property_name = "axle_width"
                    )
                ),
                make_protocol_object("controller",

                    make_protocol_number_kw(
                        &config_.controller_params.max_current_limit,
                        property_name = "max_current_limit"
                    ),
                    make_protocol_number_kw(
                        &config_.controller_params.throttle_current_limit,
                        property_name = "throttle_current_limit"
                    ),
                    make_protocol_number_kw(
                        &config_.controller_params.brake_current_limit,
                        property_name = "brake_current_limit"
                    ),
                    make_protocol_number_kw(
                        &config_.controller_params.regen_deceleration_current,
                        property_name = "regen_deceleration_current"
                    ),
                    make_protocol_number_kw(
                        &config_.controller_params.park_current_limit,
                        property_name = "park_current_limit"
                    ),
                    make_protocol_number_kw(
                        &config_.controller_params.park_vel_gain,
                        property_name = "park_vel_gain"
                    ),
                    make_protocol_number_kw(
                        &config_.controller_params.park_vel_integrator_gain,
                        property_name = "park_vel_integrator_gain"
                    ),
                    make_protocol_number_kw(
                        &config_.controller_params.park_vel_derivative_gain,
                        property_name = "park_vel_derivative_gain"
                    ),
                    make_protocol_number_kw(
                        &config_.controller_params.drive_vel_gain,
                        property_name = "drive_vel_gain"
                    ),
                    make_protocol_number_kw(
                        &config_.controller_params.drive_vel_integrator_gain,
                        property_name = "drive_vel_integrator_gain"
                    ),
                    make_protocol_number_kw(
                        &config_.controller_params.drive_vel_derivative_gain,
                        property_name = "drive_vel_derivative_gain"
                    )
                )
            )
        );
    };

    osThreadId thread_id_;
    volatile bool thread_id_valid_ = false;
    void start_thread();

private:
    
    void configure_gpio(bool initialize_gpio);
    void configure_motors(DriveMode_t drive_mode);
    void read_gpio(void);  

    TimerHandle_t debounce_timers[5] = { NULL };
};

DEFINE_ENUM_FLAG_OPERATORS(E_Vehicle::Error_t)

extern E_Vehicle *ev;

#endif
