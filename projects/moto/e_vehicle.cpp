#include "odrive_main.h"

#ifdef __E_VEHICLE_HPP

#define BENCH_TEST_MODE

extern E_Vehicle *ev;

E_Vehicle::E_Vehicle(Config_t& config) :
    config_(config)
{
}

static void cruise_control_debounced(void* class_pointer) {
    // Call function of the passed class instance
    reinterpret_cast<E_Vehicle*>(class_pointer)->toggle_state_cruise_control(true);
}

static void cruise_control_debounce_timer_callback( TimerHandle_t myTimer ) {
    // Get the passed class from myTimer (void * const pvTimerID)
    // This is unorthodox but it seems to work
    void * class_pointer = pvTimerGetTimerID(myTimer);

    // Call function of the passed class instance
    reinterpret_cast<E_Vehicle*>(class_pointer)->toggle_state_cruise_control(false);
}

void E_Vehicle::toggle_state_cruise_control(bool start_debounce_timer) {

    // If start_debounce_timer is false then we should execute the command immediately
    if (start_debounce_timer == false) {

        if( states_.cruise_control.engaged == false && states_.drive_mode == DRIVE_MODE_DRIVE) {
            states_.cruise_control.setpoint = axes[0]->encoder_.vel_estimate_;
            states_.cruise_control.gas_pedal_cached_value = states_.gas_pedal_value;
            states_.cruise_control.engaged = true;
        }

        else if(states_.cruise_control.engaged == true ) {
            states_.cruise_control.engaged = false;
            states_.cruise_control.setpoint = 0.0f;
        }
    } 
    // Otherwise start a debounce timer
    else if(debounce_timers[0] != NULL) {

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTimerResetFromISR(debounce_timers[0], &xHigherPriorityTaskWoken);
        xTimerStartFromISR(debounce_timers[0], &xHigherPriorityTaskWoken);
    }
}

static void parking_brake_debounced(void* class_pointer ) {
    // Call function of the passed class instance
    reinterpret_cast<E_Vehicle*>(class_pointer)->toggle_state_parking_brake(true);
}

static void parking_brake_debounce_timer_callback( TimerHandle_t myTimer ) {
    // Get the passed class from myTimer (void * const pvTimerID)
    // This is unorthodox but it seems to work
    void * class_pointer = pvTimerGetTimerID(myTimer);

    // Call function of the passed class instance
    reinterpret_cast<E_Vehicle*>(class_pointer)->toggle_state_parking_brake(false);
}

void E_Vehicle::toggle_state_parking_brake(bool start_debounce_timer) {

    // If start_debounce_timer is false then we should execute the command immediately
    if(start_debounce_timer == false) {

        if( states_.parking_brake.engaged == false && states_.drive_mode != DRIVE_MODE_PARK) {
            states_.cruise_control.engaged          = false;
            states_.parking_brake.engaged           = true;
            states_.parking_brake.cached_drive_mode = states_.drive_mode;
            states_.drive_mode                      = DRIVE_MODE_PARK;
        }
        else if(states_.parking_brake.engaged == true ) {
            states_.parking_brake.engaged = false;
            states_.drive_mode  = states_.parking_brake.cached_drive_mode;
            states_.parking_brake.cached_drive_mode = DRIVE_MODE_UNDEFINED;
        }
    }
    // Otherwise start a debounce timer
    else if(debounce_timers[1] != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xTimerResetFromISR(debounce_timers[1], &xHigherPriorityTaskWoken);
        xTimerStartFromISR(debounce_timers[1], &xHigherPriorityTaskWoken);
    }
}

bool E_Vehicle::update() {

    // Internal variables
    DriveMode_t cached_drive_mode = states_.drive_mode;

    // If we are not in EV mode there is nothing to do
    if(config_.ev_controller_enabled != true) {

        // Toss error if this mode is not enabled!
        error_ = ERROR_MODE_NOT_ENABLED;

        // Reset some important default conditions
        states_.drive_mode      = DRIVE_MODE_PARK;
        states_.last_drive_mode = DRIVE_MODE_UNDEFINED;
        
        // Unsubscribe GPIO pins
        configure_gpio(false);

        // Break
        return false;
    }
    
    // Otherwise, let's roll! First clear errors...
    error_ = ERROR_NONE;

    // Configure IO pins for brake, gas, etc  
    configure_gpio(true);
 
    // How many axis do we have to configure?
    switch(config_.physics.drive_config) {

        case DRIVE_CONFIG_1WD:
            states_.number_of_active_axes = 1;  
            break;
        case DRIVE_CONFIG_2WD_INLINE:
            states_.number_of_active_axes = 2;
            break;
        default:
            // For now just 2 wheels inline supported
            error_ |= ERROR_FEATURE_UNIMPLEMENTED;
            return 0;
    }

    // Determine our axis indicies 
    states_.master_axis         = config_.physics.master_axis;
    states_.subordinate_axis    = (1 - states_.master_axis);

    // Read relevant IO pins to determine the vehicle state
    read_gpio();

    // If brake is engaged then turn the cruise control off
    if(states_.brake_pedal_value > 0.0f) {

        states_.cruise_control.engaged = false;
        states_.cruise_control.setpoint = 0.0f;

        if(states_.brake_pedal_value > 0.9f) {

            if( abs(axes[states_.master_axis]->encoder_.vel_estimate_) < 10.0f &&
                states_.brake_locked == false) {
                states_.brake_locked_last_state = false;
                states_.brake_locked = true;
            }
        }

        else if(states_.brake_locked == true) {
            states_.brake_locked            = false;
            states_.brake_locked_last_state = true;
        }

    }

    // Check if we shifted and if true, then configure motors
    if(states_.brake_locked != states_.brake_locked_last_state) {
        states_.brake_locked_last_state = states_.brake_locked;

        if(states_.brake_locked == true) {
            configure_motors(DRIVE_MODE_PARK);
        }
        else {
            configure_motors(cached_drive_mode);
        }
    }
    else if(cached_drive_mode != states_.last_drive_mode) {    
        configure_motors(cached_drive_mode);
    }
    

    // State machine for shifter positions! ----------------------------------------
    // State where the motor is immobilized
    if (cached_drive_mode == DRIVE_MODE_PARK) {      
        // Maybe do something here?
    }

    // States where the motor is moving in some powered way
    else if( cached_drive_mode == DRIVE_MODE_DRIVE ||
        cached_drive_mode == DRIVE_MODE_REVERSE ||
        cached_drive_mode == DRIVE_MODE_NEUTRAL ) {

        // Configure control loop for first motor
        if( config_.physics.drive_config == DRIVE_CONFIG_1WD || 
            config_.physics.drive_config == DRIVE_CONFIG_2WD_INLINE ) {   
            // Update controller setpoints
            // Brake
            axes[states_.master_axis]->controller_.brake_setpoint_ = states_.brake_pedal_value; 

            // Throttle
            switch(cached_drive_mode) {

                case DRIVE_MODE_DRIVE:    
                    
                    #ifndef BENCH_TEST_MODE
                    if( states_.cruise_control.engaged &&
                        states_.gas_pedal_value < states_.cruise_control.gas_pedal_cached_value) {
                        axes[states_.master_axis]->controller_.config_.vel_limit = states_.cruise_control.setpoint;
                        axes[states_.master_axis]->controller_.throttle_setpoint_ = 1.0f;
                    }
                    else {
                        axes[states_.master_axis]->controller_.config_.vel_limit  = 20000.0f;
                        axes[states_.master_axis]->controller_.throttle_setpoint_ = states_.gas_pedal_value;
                    }
                    #else
                        axes[states_.master_axis]->controller_.config_.vel_limit  = 200.0f;
                        axes[states_.master_axis]->controller_.throttle_setpoint_ = states_.gas_pedal_value;
                    #endif

                    break;
                case DRIVE_MODE_REVERSE:
                    axes[states_.master_axis]->controller_.throttle_setpoint_ = -states_.gas_pedal_value;
                    break;
                default:
                    axes[states_.master_axis]->controller_.throttle_setpoint_ = 0.0f;
                    break;
            }
        }

        // Configure control loop for second motor
        if(config_.physics.drive_config == DRIVE_CONFIG_2WD_INLINE) {

            // Update controller setpoints
            // Brake
            axes[states_.subordinate_axis]->controller_.brake_setpoint_ = states_.brake_pedal_value; 

            // Throttle
            switch(cached_drive_mode) {
                case DRIVE_MODE_DRIVE:
                    if(states_.cruise_control.engaged) {
                        axes[states_.subordinate_axis]->controller_.config_.vel_limit   = states_.cruise_control.setpoint;
                        axes[states_.subordinate_axis]->controller_.throttle_setpoint_  = 1.0f;
                    }
                    else {
                        axes[states_.subordinate_axis]->controller_.config_.vel_limit   = 20000;
                        axes[states_.subordinate_axis]->controller_.throttle_setpoint_  = states_.gas_pedal_value;
                    }
                    break;
                case DRIVE_MODE_REVERSE:
                    axes[states_.subordinate_axis]->controller_.throttle_setpoint_      = -states_.gas_pedal_value;
                    break;
                default:
                    axes[states_.subordinate_axis]->controller_.throttle_setpoint_      = 0.0f;
                    break;
            }

            // Constrain second motor velocity to the primary motor velocity (to prevent skidding)
            axes[states_.subordinate_axis]->controller_.config_.vel_limit
                = axes[states_.master_axis]->encoder_.vel_estimate_;
        }
    }

    // Cache drive mode so we don't run motor configuration routines again
    states_.last_drive_mode = cached_drive_mode;

    return 1;
}


// Private --------------


void E_Vehicle::configure_gpio(bool initialize_gpio) {
    if(initialize_gpio == true && states_.configured_gpio_pins_for_ev == false) {
        // Init GPIO pins
        GPIO_set_to_analog(GPIO_3_GPIO_Port, GPIO_3_Pin);
        GPIO_set_to_analog(GPIO_4_GPIO_Port, GPIO_4_Pin);

        #ifndef BENCH_TEST_MODE
        // Attach GPIO interrupts
        GPIO_subscribe(GPIO_5_GPIO_Port, GPIO_5_Pin, GPIO_PULLDOWN, cruise_control_debounced, this, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
        GPIO_subscribe(GPIO_6_GPIO_Port, GPIO_6_Pin, GPIO_PULLDOWN, parking_brake_debounced, this, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY + 1);
        #endif

        // Create debounce timers
        debounce_timers[0] = xTimerCreate( "cc_t", 200, pdFALSE, this, cruise_control_debounce_timer_callback);
        debounce_timers[1] = xTimerCreate( "pb_t", 200, pdFALSE, this, parking_brake_debounce_timer_callback);

        // Set global flag
        states_.configured_gpio_pins_for_ev = true;
    }

    else if(initialize_gpio == false && states_.configured_gpio_pins_for_ev == true) {
        // Destroy timers
        for(int i = 0; i < 5; i++) {
            //if(debounce_timers[i] != NULL) xTimerDelete(debounce_timers[i], 0);
        }

        // Unattach GPIO
        GPIO_unsubscribe(GPIO_5_GPIO_Port, GPIO_5_Pin);
        GPIO_unsubscribe(GPIO_6_GPIO_Port, GPIO_6_Pin);

        // Clear global flag
        states_.configured_gpio_pins_for_ev = false;
    }
}

void E_Vehicle::read_gpio(void) {

    float gas_adc_voltage_reading   = get_adc_voltage(GPIO_3_GPIO_Port, GPIO_3_Pin) / 3.3f;
    float brake_adc_voltage_reading = get_adc_voltage(GPIO_4_GPIO_Port, GPIO_4_Pin) / 3.3f;

    float gas_pedal_mapped = MACRO_MAP( gas_adc_voltage_reading, 
                                    config_.gas_pedal_adc_min_voltage, 
                                    config_.gas_pedal_adc_max_voltage, 
                                    0.0f, 1.0f );

    float brake_pedal_mapped = MACRO_MAP( brake_adc_voltage_reading, 
                                        config_.brake_pedal_adc_min_voltage, 
                                        config_.brake_pedal_adc_max_voltage, 
                                        0.0f, 1.0f );

    #ifndef BENCH_TEST_MODE
    states_.gas_pedal_value = constrain(gas_pedal_mapped, 0.0f, 1.0f);
    states_.brake_pedal_value = constrain(brake_pedal_mapped, 0.0f, 1.0f);
    #else
    states_.gas_pedal_value = 0.02f;
    states_.brake_pedal_value = 0.0f;
    #endif
}

void E_Vehicle::configure_motors(DriveMode_t drive_mode) {
    // Reconfigure all motors

    switch(drive_mode) {
        case DRIVE_MODE_DRIVE:
        case DRIVE_MODE_NEUTRAL:
        case DRIVE_MODE_REVERSE:
            for(int i = 0; i < states_.number_of_active_axes; i++) {
                axes[i]->motor_.config_.current_control_bandwidth  = 100;
                axes[i]->encoder_.config_.bandwidth                = 100;

                axes[i]->motor_.config_.current_lim                = config_.controller_params.max_current_limit;

                axes[i]->controller_.config_.throttle_acceleration_current_limit    = config_.controller_params.throttle_current_limit;
                axes[i]->controller_.config_.throttle_brake_current_limit           = config_.controller_params.brake_current_limit;
                axes[i]->controller_.config_.throttle_regen_deceleration_current    = config_.controller_params.regen_deceleration_current;

                axes[i]->controller_.config_.vel_gain              = config_.controller_params.drive_vel_gain;
                axes[i]->controller_.config_.vel_integrator_gain   = config_.controller_params.drive_vel_integrator_gain;
                axes[i]->controller_.config_.vel_derivative_gain   = config_.controller_params.drive_vel_derivative_gain;

                axes[i]->controller_.config_.control_mode          = Controller::CTRL_MODE_THROTTLE_CONTROL; 
                axes[i]->requested_state_                          = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;             
            }
            break;

        case DRIVE_MODE_PARK:
        default:
            for(int i = 0; i < states_.number_of_active_axes; i++) {
                // Set parameters for effective parking braking 
                axes[i]->encoder_.config_.bandwidth                = 100;
                axes[i]->motor_.config_.current_control_bandwidth  = 100;

                axes[i]->motor_.config_.current_lim                         = config_.controller_params.park_current_limit;
                axes[i]->controller_.config_.throttle_brake_current_limit   = config_.controller_params.park_current_limit;
                axes[i]->controller_.config_.setpoints_in_cpr               = true;

                axes[i]->controller_.throttle_setpoint_            = 0.0f;
                axes[i]->controller_.brake_setpoint_               = 0.0f;
                axes[i]->controller_.vel_setpoint_                 = 0.0f;
                
                axes[i]->controller_.config_.control_mode          = Controller::CTRL_MODE_VELOCITY_CONTROL;
                axes[i]->requested_state_                          = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
                
                axes[i]->controller_.config_.vel_gain              = config_.controller_params.park_vel_gain;
                axes[i]->controller_.config_.vel_integrator_gain   = config_.controller_params.park_vel_integrator_gain;
                axes[i]->controller_.config_.vel_derivative_gain   = config_.controller_params.park_vel_derivative_gain;        
            }
            break;
    }
}


static void e_vehicle_update_wrapper(void *) {
    uint32_t cnt = 0;

    bool started_phase_1 = false;
    bool started_phase_2 = false;

    axes[0]->motor_.config_.current_lim = 10.0f;

    #if defined(HFI_DEBUGGING_INSPECT_ESTIMATOR) || defined(HFI_DEBUGGING_INSPECT_FUSION) || defined(HFI_DEBUGGING_INSPECT_DRIVE_CURRENT)
        //axes[0]->sensorless_estimator_.config_.hfi_calibrated = 0;  
    #endif 


    while(true) {
        ev->update();
        osDelay(10);

        cnt++;
        if(cnt == 10) {
            axes[0]->motor_.is_calibrated_ = true; 
            //#if defined(HFI_DEBUGGING_INSPECT_ESTIMATOR) || defined(HFI_DEBUGGING_INSPECT_FUSION) || defined(HFI_DEBUGGING_INSPECT_DRIVE_CURRENT)
                //axes[0]->motor_.is_calibrated_ = false; 
                //axes[0]->requested_state_ = Axis::AXIS_STATE_MOTOR_CALIBRATION;
            //#endif
            started_phase_1 = true;
        }

        else if(started_phase_1 && axes[0]->motor_.is_calibrated_ == true && !started_phase_2) {
            //axes[0]->requested_state_ = Axis::AXIS_STATE_HFI_CALIBRATION;

            axes[0]->sensorless_estimator_.config_.hfi_enabled = 1;

            //axes[0]->config_.lockin.vel = 0.5f;
            //axes[0]->config_.lockin.current = 3.0f;
            //axes[0]->requested_state_ = Axis::AXIS_STATE_LOCKIN_SPIN;
            //axes[0]->config_.lockin.vel = 0.0f;
            //axes[0]->config_.lockin.current = 0.0f;  
            
            //axes[0]->controller_.config_.control_mode          = Controller::CTRL_MODE_VELOCITY_CONTROL;
            //axes[0]->controller_.vel_setpoint_                 = 1.0f;//-1.0f;
            //axes[0]->controller_.pos_setpoint_                 = 0;//-1.0f;

            //axes[0]->controller_.config_.control_mode        = Controller::CTRL_MODE_CURRENT_CONTROL;
            //axes[0]->controller_.current_setpoint_           = 0.0f;

            axes[0]->requested_state_ = Axis::AXIS_STATE_SENSORLESS_CONTROL;

            axes[0]->controller_.config_.control_mode          = Controller::CTRL_MODE_VELOCITY_CONTROL;
            axes[0]->motor_.config_.current_lim = 20.0f;
            axes[0]->controller_.vel_setpoint_                 = 1.0f;
            axes[0]->controller_.pos_setpoint_                 = 0;
            axes[0]->config_.lockin.vel                        = 1.0f;
            axes[0]->controller_.vel_setpoint_                 = 1.0f;

            //axes[0]->controller_.current_setpoint_           = 0.0f;

            //axes[0]->controller_.config_.control_mode          = Controller::CTRL_MODE_VELOCITY_CONTROL;
            //axes[0]->controller_.vel_setpoint_                 = -1.0f;


            /*
            // Set parameters for effective parking braking 
            axes[0]->encoder_.config_.bandwidth                = 100;
            axes[0]->motor_.config_.current_control_bandwidth  = 100;

            axes[0]->motor_.config_.current_lim                         = ev->config_.controller_params.park_current_limit;
            axes[0]->controller_.config_.throttle_brake_current_limit   = ev->config_.controller_params.park_current_limit;
            axes[0]->controller_.config_.setpoints_in_cpr               = true;

            axes[0]->controller_.throttle_setpoint_            = 0.0f;
            axes[0]->controller_.brake_setpoint_               = 0.0f;
            axes[0]->controller_.vel_setpoint_                 = 0.0f;
            
            axes[0]->controller_.config_.control_mode          = Controller::CTRL_MODE_VELOCITY_CONTROL;
            axes[0]->requested_state_                          = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
            
            axes[0]->controller_.config_.vel_gain              = ev->config_.controller_params.park_vel_gain;
            axes[0]->controller_.config_.vel_integrator_gain   = ev->config_.controller_params.park_vel_integrator_gain;
            axes[0]->controller_.config_.vel_derivative_gain   = ev->config_.controller_params.park_vel_derivative_gain;  
            */

           started_phase_2 = true;
        }
    }
}

void E_Vehicle::start_thread() {
    osThreadDef(e_vehicle_thread, e_vehicle_update_wrapper, config_.thread_priority, 0, 512);

    thread_id_ = osThreadCreate(osThread(e_vehicle_thread), this);

    __asm__ ("nop");
}

#endif