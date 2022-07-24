
#include <stdlib.h>
#include <functional>
#include "gpio.h"

#include "utils.h"
#include "odrive_main.h"


// =========================================================================
Axis::Axis(const AxisHardwareConfig_t& hw_config,
           Config_t& config,
           Encoder& encoder,
           SensorlessEstimator& sensorless_estimator,
           Controller& controller,
           Motor& motor,
           TrapezoidalTrajectory& trap)
    : hw_config_(hw_config),
      config_(config),
      encoder_(encoder),
      sensorless_estimator_(sensorless_estimator),
      controller_(controller),
      motor_(motor),
      trap_(trap)
{
    encoder_.axis_ = this;
    sensorless_estimator_.axis_ = this;
    controller_.axis_ = this;
    motor_.axis_ = this;
    trap_.axis_ = this;

    decode_step_dir_pins();
    update_watchdog_settings();
}


// =========================================================================
static void step_cb_wrapper(void* ctx) {
    reinterpret_cast<Axis*>(ctx)->step_cb();
}


// =========================================================================
// @brief Sets up all components of the axis,
// such as gate driver and encoder hardware.
void Axis::setup() {
    encoder_.setup();
    motor_.setup();
}


// =========================================================================
static void run_state_machine_loop_wrapper(void* ctx) {
    reinterpret_cast<Axis*>(ctx)->run_state_machine_loop();
    reinterpret_cast<Axis*>(ctx)->thread_id_valid_ = false;
}


// =========================================================================
// @brief Starts run_state_machine_loop in a new thread
void Axis::start_thread() {
    osThreadDef(axis_thread, run_state_machine_loop_wrapper, hw_config_.thread_priority, 0, 5*512 + 256);
    thread_id_ = osThreadCreate(osThread(axis_thread), this);
    thread_id_valid_ = true;
}


// =========================================================================
// @brief Unblocks the control loop thread.
// This is called from the current sense interrupt handler.
void Axis::signal_current_meas() {
    if (thread_id_valid_)
        osSignalSet(thread_id_, M_SIGNAL_PH_CURRENT_MEAS);
}


// =========================================================================
// @brief Blocks until a current measurement is completed
// @returns True on success, false otherwise
bool Axis::wait_for_current_meas() {
    return osSignalWait(M_SIGNAL_PH_CURRENT_MEAS, PH_CURRENT_MEAS_TIMEOUT).status == osEventSignal;
}


// =========================================================================
// step/direction interface
void Axis::step_cb() {
    if (step_dir_active_) {
        GPIO_PinState dir_pin = HAL_GPIO_ReadPin(dir_port_, dir_pin_);
        float dir = (dir_pin == GPIO_PIN_SET) ? 1.0f : -1.0f;
        controller_.pos_setpoint_ += dir * config_.counts_per_step;
    }
};


// =========================================================================
void Axis::load_default_step_dir_pin_config(
        const AxisHardwareConfig_t& hw_config, Config_t* config) {
    config->step_gpio_pin = hw_config.step_gpio_pin;
    config->dir_gpio_pin = hw_config.dir_gpio_pin;
}


// =========================================================================
void Axis::decode_step_dir_pins() {
    step_port_ = get_gpio_port_by_pin(config_.step_gpio_pin);
    step_pin_ = get_gpio_pin_by_pin(config_.step_gpio_pin);
    dir_port_ = get_gpio_port_by_pin(config_.dir_gpio_pin);
    dir_pin_ = get_gpio_pin_by_pin(config_.dir_gpio_pin);
}


// =========================================================================
// @brief: Setup the watchdog reset value from the configuration watchdog timeout interval. 
void Axis::update_watchdog_settings() {

    if(config_.watchdog_timeout <= 0.0f) { // watchdog disabled 
        watchdog_reset_value_ = 0;
    } 
    
    else if(config_.watchdog_timeout >= UINT32_MAX / (current_meas_hz+1)) { //overflow! 
        watchdog_reset_value_ = UINT32_MAX;
    } 
    
    else {
        watchdog_reset_value_ = static_cast<uint32_t>(config_.watchdog_timeout * current_meas_hz);
    }

    // Do a feed to avoid instant timeout
    watchdog_feed();
}


// =========================================================================
// @brief (de)activates step/dir input
void Axis::set_step_dir_active(bool active) {

    if (active) {
        // Set up the direction GPIO as input
        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = dir_pin_;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(dir_port_, &GPIO_InitStruct);

        // Subscribe to rising edges of the step GPIO
        GPIO_subscribe(step_port_, step_pin_, GPIO_PULLDOWN,
                step_cb_wrapper, this, 0);

        step_dir_active_ = true;
    } 

    else {
        step_dir_active_ = false;

        // Unsubscribe from step GPIO
        GPIO_unsubscribe(step_port_, step_pin_);
    }
}

// =========================================================================
// @brief Do axis level checks and call subcomponent do_checks
// Returns true if everything is ok.
bool Axis::do_checks() {

    if (!brake_resistor_armed) {
        error_ |= ERROR_BRAKE_RESISTOR_DISARMED;
    }

    if ((current_state_ != AXIS_STATE_IDLE) && (motor_.armed_state_ == Motor::ARMED_STATE_DISARMED)) {
        // motor got disarmed in something other than the idle loop
        error_ |= ERROR_MOTOR_DISARMED;
    }
    
    if (!(vbus_voltage >= board_config.dc_bus_undervoltage_trip_level)) {
        error_ |= ERROR_DC_BUS_UNDER_VOLTAGE;
    }
    
    if (!(vbus_voltage <= board_config.dc_bus_overvoltage_trip_level)) {
        error_ |= ERROR_DC_BUS_OVER_VOLTAGE;
    }

    // Sub-components should use set_error which will propegate to this error_
    motor_.do_checks();
    encoder_.do_checks();
    // sensorless_estimator_.do_checks();
    // controller_.do_checks();

    return check_for_errors();
}


// =========================================================================
// @brief Update all esitmators
bool Axis::do_updates() {
    // Sub-components should use set_error which will propegate to this error_
    encoder_.update();

    sensorless_estimator_.update();

    return check_for_errors();
}


// =========================================================================
// @brief Feed the watchdog to prevent watchdog timeouts.
void Axis::watchdog_feed() {
    watchdog_current_value_ = watchdog_reset_value_;
}


// =========================================================================
// @brief Check the watchdog timer for expiration. Also sets the watchdog error bit if expired. 
bool Axis::watchdog_check() {
    // reset value = 0 means watchdog disabled. 
    if(watchdog_reset_value_ == 0) return true;

    // explicit check here to ensure that we don't underflow back to UINT32_MAX
    if(watchdog_current_value_ > 0) {
        watchdog_current_value_--;
        return true;
    } 
    
    else {
        error_ |= ERROR_WATCHDOG_TIMER_EXPIRED;
        return false;
    }
}


// =========================================================================
bool Axis::run_lockin_spin(bool skip_accelerate = 0) {

    float distance, finish_distance, phase, vel;

    // Function of states to check if we are done
    auto spin_done = [&](bool vel_override = false) -> bool {
        bool done = false;

        if(config_.lockin.finish_on_flag) {
            done = done || lockin_finish_now_flag_;
        }
        if (config_.lockin.finish_on_vel || vel_override) {
            done = done || fast_absf(vel) >= fast_absf(config_.lockin.vel);
        }
        if (config_.lockin.finish_on_distance) {
            done = done || fast_absf(distance) >= fast_absf(finish_distance);
        }
        if (config_.lockin.finish_on_enc_idx) {
            done = done || encoder_.index_found_;
        }
        return done;
    };

    if(!skip_accelerate) {

        // Spiral up current for softer rotor lock-in
        lockin_state_ = LOCKIN_STATE_RAMP;

        float x = 0.0f;

        run_control_loop([&]() {
            phase = wrap_pm_pi(config_.lockin.ramp_distance * x);
            float I_mag = config_.lockin.current * x;
            x += current_meas_period / config_.lockin.ramp_time;
            
            if (!motor_.update(I_mag, 0.0f, phase, 0.0f)) {
                return false;
            }

            return x < 1.0f;
        });

        // Spin states
        distance = config_.lockin.ramp_distance;
        phase = wrap_pm_pi(distance);
        finish_distance = config_.lockin.finish_distance;
        vel = distance / config_.lockin.ramp_time;

        // Accelerate
        lockin_state_ = LOCKIN_STATE_ACCELERATE;
        run_control_loop([&]() {
            vel += config_.lockin.accel * current_meas_period;
            distance += vel * current_meas_period;
            phase = wrap_pm_pi(phase + vel * current_meas_period);

            if (!motor_.update(config_.lockin.current, 0.0f, phase, vel)) {
                return false;
            }
            
            return !spin_done(true); //vel_override to go to next phase
        });
    }
    
    else {
        phase = wrap_pm_pi(config_.lockin.initial_phase);
        distance = 0;//wrap_pm_pi(config_.lockin.initial_phase);
        finish_distance = config_.lockin.finish_distance;// + config_.lockin.initial_phase;
    }

    if (!encoder_.index_found_) {
        encoder_.set_idx_subscribe(true);
    }

    // Constant speed
    if (!spin_done()) {
        lockin_state_ = LOCKIN_STATE_CONST_VEL;
        vel = config_.lockin.vel; // reset to actual specified vel to avoid small integration error

        run_control_loop([&]() {
            distance += vel * current_meas_period;

            phase = wrap_pm_pi(phase + vel * current_meas_period);

            if (!motor_.update(config_.lockin.current, 0.0f, phase, vel)) {
                return false;
            }

            if(spin_done()) {
                //motor_.saved_steady_state_v_current_control_integral_d_ = motor_.current_control_.v_current_control_integral_d;
                //motor_.saved_steady_state_v_current_control_integral_q_ = motor_.current_control_.v_current_control_integral_q;
                return false;  
            }
            
            else {
                return true;
            }
        });
    }

    config_.lockin.initial_phase = 0.0f;
    config_.lockin.finish_on_distance = 0;
    config_.lockin.finish_on_vel = 0;
    config_.lockin.finish_on_enc_idx = 0;
    config_.lockin.finish_on_flag = 0;
    lockin_finish_now_flag_ = false;

    lockin_state_ = LOCKIN_STATE_INACTIVE;
    return check_for_errors();
}


// =========================================================================
// Note run_sensorless_control_loop and run_closed_loop_control_loop are very similar 
// and differ only in where we get the estimate from.
bool Axis::run_sensorless_control_loop() {

    //controller_.config_.vel_gain = 0.2f;
    //controller_.vel_setpoint_ = 0.0f;
    //sensorless_estimator_.config_.hfi_max_velocity = 60.0f;
    //sensorless_estimator_.config_.hfi_vel_hysteresis = 0.1f;

    run_control_loop([this]() {
        // Check if control mode is an absolute one and toss error if yes
        /*
        if (controller_.config_.control_mode == Controller::CTRL_MODE_POSITION_CONTROL ||
            controller_.config_.control_mode == Controller::CTRL_MODE_TRAJECTORY_CONTROL) {
            return error_ |= ERROR_POS_CTRL_DURING_SENSORLESS, false;
        }
        */

        // Note that all estimators are updated in the loop prefix in run_control_loop
        float d_current_setpoint;
        float q_current_setpoint;

        // Do we have high frequency injection enabled?
        if(sensorless_estimator_.config_.hfi_enabled == true) {

            // If we have HFI enabled, then atlow speeds we will use the motor as a reluctance motor 
            // (direct-axis aiglined), and the HFI signal will tell usthe misalignment between our 
            // rotor and our direct axis. If the misailgnment is high, then we can increase the direct
            // axis current (Id) to enforce greater alignment between the rotor and the fundamental
            // magnetic field current. 

            // Get the magnitude of our velocity
            float velocity_magnitude_ = fast_absf(sensorless_estimator_.vel_estimate_filtered_);

            // If we are not in direct mode, then test here to see if the rotor speed has fallen below 
            // hfi_max_velocity, and its hysteresis factor            
            if( controller_.direct_mode_ == false 
            && velocity_magnitude_ < sensorless_estimator_.config_.hfi_max_velocity - sensorless_estimator_.config_.hfi_max_velocity * sensorless_estimator_.config_.hfi_vel_hysteresis) {
                
                // If it has, then we should enter direct mode control...
                controller_.direct_mode_ = true;

                // Enable the HF inection...
                sensorless_estimator_.hfi_engaged = 1;

                // And set our direct mode phase to the best estimate of our rotor phase, courtesy
                // from our phase estimator
                controller_.direct_mode_phase_ = wrap_pm_pi(sensorless_estimator_.phase_);
            }

            // If we are already in direct mode, then test to see if our velocity is greater than our 
            // hfi_max_velocity. If it is, then we can turn off the HF injection and use our BEMF estimate
            // without any help.
            else if(
                 controller_.direct_mode_ == true 
              && velocity_magnitude_ > sensorless_estimator_.config_.hfi_max_velocity) {
                
                // Disable the HF injection...
                sensorless_estimator_.hfi_engaged = 0;
                
                // Leave direct mode, switch to Q axis control...
                controller_.direct_mode_ = false;
            }
        }

        // If HFI is not enabled then stick with Q axis control only
        else {
            // Disable HF injection... 
            sensorless_estimator_.hfi_engaged = 0;

            // Don't use direct mode...
            controller_.direct_mode_ = false;
        }

        // Are we using direct axis control?
        if(controller_.direct_mode_){

            // Yes, set our direct mode velocity to our velocity setpoint
            // @todo: gracefully change this using motor dynamics estimate

            controller_.direct_mode_vel_ = controller_.vel_setpoint_;
            
            // Estimate our direct mode position by integrating direct mode velocity by one time step
            controller_.direct_mode_position_ += controller_.direct_mode_vel_ * current_meas_period;

            // Convert direct mode position to phase by wrapping around 2 pi
            controller_.direct_mode_phase_    = wrap_pm_pi(controller_.direct_mode_phase_ + controller_.direct_mode_vel_ * current_meas_period);
            
            // Using this line instead makes spaceship sounds after a while then faults
            //controller_.direct_mode_phase_      = wrap_pm_pi(controller_.direct_mode_position_);

            // Try to update our controller with new parameters
            if (!controller_.update(
                controller_.direct_mode_position_, 
                controller_.direct_mode_vel_, 
                controller_.direct_mode_phase_, 
                &d_current_setpoint, 
                &q_current_setpoint)
                ) {
                // Return if error
                return error_ |= ERROR_CONTROLLER_FAILED, false;
            }

            // Now try to update our motor with new parameters as set by the controller
            if (!motor_.update(
                    sensorless_estimator_.config_.hfi_quiescent_current + controller_.direct_mode_direct_current_, 
                    0.0f, // Zero Iq 
                    controller_.direct_mode_phase_ , 
                    controller_.direct_mode_vel_)
                ) {
                // Return if error
                return false;
            }
        }

        // If we are not in direct axis control mode...
        else {

            // Update our controller with estimates from our BEMF flux estimator
            if (!controller_.update(
                    sensorless_estimator_.pll_pos_, 
                    sensorless_estimator_.vel_estimate_, 
                    sensorless_estimator_.phase_, 
                    &d_current_setpoint, 
                    &q_current_setpoint)
                ) {
                // Return if error
                return error_ |= ERROR_CONTROLLER_FAILED, false;
            }

            // Now try to update our motor with new parameters as set by the controller
            if (!motor_.update(
                    sensorless_estimator_.config_.hfi_quiescent_current + d_current_setpoint, 
                    q_current_setpoint, 
                    sensorless_estimator_.phase_, 
                    sensorless_estimator_.vel_estimate_)
                ) {
                // Return if error
                return false; // set_error should update axis.error_
            }
        }
        
        return true;
    });

    return check_for_errors();
}


// =========================================================================
bool Axis::run_closed_loop_control_loop() {

    // To avoid any transient on startup, we intialize the setpoint to be the current position
    controller_.pos_setpoint_ = encoder_.pos_estimate_;
    set_step_dir_active(config_.enable_step_dir);

    run_control_loop([this]() {

        // Note that all estimators are updated in the loop prefix in run_control_loop
        float d_current_setpoint;
        float q_current_setpoint;

        float phase_vel  = 2.0f * M_PI * encoder_.vel_estimate_ / (float)encoder_.config_.cpr * motor_.config_.pole_pairs;
        
        if (!controller_.update(encoder_.pos_estimate_, encoder_.vel_estimate_, encoder_.phase_, &d_current_setpoint, &q_current_setpoint)) {
            return error_ |= ERROR_CONTROLLER_FAILED, false; //TODO: Make controller.set_error
        }

        if (!motor_.update(d_current_setpoint, q_current_setpoint, encoder_.phase_, phase_vel)) {
            return false; // set_error should update axis.error_
        }

        return true;
    });

    set_step_dir_active(false);
    return check_for_errors();
}


// =========================================================================
bool Axis::run_idle_loop() {
    // run_control_loop ignores missed modulation timing updates
    // if and only if we're in AXIS_STATE_IDLE
    safety_critical_disarm_motor_pwm(motor_);
    run_control_loop([this](){
        return true;
    });
    return check_for_errors();
}


// =========================================================================
// Infinite loop that does calibration and enters main control loop as appropriate
void Axis::run_state_machine_loop() {

    // Allocate the map for anti-cogging algorithm and initialize all values to 0.0f
    // TODO: Move this somewhere else
    // TODO: respect changes of CPR
    int encoder_cpr = encoder_.config_.cpr;
    controller_.anticogging_.cogging_map = (float*)malloc(encoder_cpr * sizeof(float));
    if (controller_.anticogging_.cogging_map != NULL) {
        for (int i = 0; i < encoder_cpr; i++) {
            controller_.anticogging_.cogging_map[i] = 0.0f;
        }
    }

    // arm!
    motor_.arm();
    
    for (;;) {
        // Load the task chain if a specific request is pending
        if (requested_state_ != AXIS_STATE_UNDEFINED) {

            // Clear errors
            if(requested_state_ != AXIS_STATE_IDLE && requested_state_ != AXIS_STATE_UNDEFINED) {
                motor_.error_ = Motor::ERROR_NONE;
                error_ = ERROR_NONE;
                controller_.error_ = Controller::ERROR_NONE;
                sensorless_estimator_.error_ = SensorlessEstimator::ERROR_NONE;
            };

            size_t pos = 0;

            if (requested_state_ == AXIS_STATE_STARTUP_SEQUENCE) {
                if (config_.startup_motor_calibration) {
                    task_chain_[pos++] = AXIS_STATE_MOTOR_CALIBRATION;
                }
                if (config_.startup_encoder_index_search && encoder_.config_.use_index) {
                    task_chain_[pos++] = AXIS_STATE_ENCODER_INDEX_SEARCH;
                }
                if (config_.startup_encoder_offset_calibration) {
                    task_chain_[pos++] = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
                }
                if (config_.startup_closed_loop_control) {
                    task_chain_[pos++] = AXIS_STATE_CLOSED_LOOP_CONTROL;
                }
                else if (config_.startup_sensorless_control) {
                    task_chain_[pos++] = AXIS_STATE_SENSORLESS_CONTROL;
                }
                task_chain_[pos++] = AXIS_STATE_IDLE;
            } 
            else if (requested_state_ == AXIS_STATE_FULL_CALIBRATION_SEQUENCE) {
                task_chain_[pos++] = AXIS_STATE_MOTOR_CALIBRATION;
                if (encoder_.config_.use_index) {
                    task_chain_[pos++] = AXIS_STATE_ENCODER_INDEX_SEARCH;
                }
                task_chain_[pos++] = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
                task_chain_[pos++] = AXIS_STATE_IDLE;
            } 
            else if (requested_state_ != AXIS_STATE_UNDEFINED) {
                task_chain_[pos++] = requested_state_;
                task_chain_[pos++] = AXIS_STATE_IDLE;
            }

            // TODO: bounds checking
            task_chain_[pos++] = AXIS_STATE_UNDEFINED; 
            requested_state_ = AXIS_STATE_UNDEFINED;

            // Auto-clear any invalid state error
            error_ &= ~ERROR_INVALID_STATE;
        }

        // Note that current_state is a reference to task_chain_[0]

        // Run the specified state
        // Handlers should exit if requested_state != AXIS_STATE_UNDEFINED
        bool status;

        switch (current_state_) {
            case AXIS_STATE_MOTOR_CALIBRATION: {
                status = motor_.run_calibration();
            } break;

            case AXIS_STATE_ENCODER_INDEX_SEARCH: {
                if (!motor_.is_calibrated_) {
                    goto invalid_state_label;
                }
                if (encoder_.config_.idx_search_unidirectional && motor_.config_.direction==0) {
                    goto invalid_state_label;
                }
                status = encoder_.run_index_search();
            } break;

            case AXIS_STATE_ENCODER_DIR_FIND: {
                if (!motor_.is_calibrated_) {
                    goto invalid_state_label;
                }
                status = encoder_.run_direction_find();
            } break;

            case AXIS_STATE_ENCODER_OFFSET_CALIBRATION: {
                if (!motor_.is_calibrated_) {
                    goto invalid_state_label;
                }
                status = encoder_.run_offset_calibration();
            } break;

            case AXIS_STATE_HFI_CALIBRATION: {
                if (!motor_.is_calibrated_) {
                    goto invalid_state_label;
                }                
                //todo make configurable
                status = sensorless_estimator_.measure_hfi_intrinsics();
                if(status) {
                    //status = sensorless_estimator_.measure_q_current_offsets(15.0f, 15.0f);
                }
            } break;

            case AXIS_STATE_LOCKIN_SPIN: {
                if (!motor_.is_calibrated_ || motor_.config_.direction==0) {
                    goto invalid_state_label;
                }
                status = run_lockin_spin(false);
            } break;

            case AXIS_STATE_SENSORLESS_CONTROL: {
                if (!motor_.is_calibrated_ || motor_.config_.direction==0) {
                    goto invalid_state_label;
                }
                //config_.lockin.finish_on_vel = true;
                //status = run_lockin_spin(false); // TODO: restart if desired
                status = 1;
                if (status) {
                    // call to controller.reset() that happend when arming means that vel_setpoint
                    // is zeroed. So we make the setpoint the spinup target for smooth transition.
                    //controller_.vel_setpoint_ = config_.lockin.vel;
                    status = run_sensorless_control_loop();
                }
            } break;

            case AXIS_STATE_CLOSED_LOOP_CONTROL: {
                if (!motor_.is_calibrated_ || motor_.config_.direction==0) {
                    goto invalid_state_label;
                }
                if (!encoder_.is_ready_) {
                    goto invalid_state_label;
                }
                status = run_closed_loop_control_loop();
            } break;

            case AXIS_STATE_IDLE: {
                run_idle_loop();
                status = motor_.arm(); // done with idling - try to arm the motor
            } break;

            default:
            invalid_state_label:
                error_ |= ERROR_INVALID_STATE;
                status = false; // this will set the state to idle
                break;
        }

        // If the state failed, go to idle, else advance task chain
        if (!status)
            current_state_ = AXIS_STATE_IDLE;
        else
            memcpy(task_chain_, task_chain_ + 1, sizeof(task_chain_) - sizeof(task_chain_[0]));
    }
}
